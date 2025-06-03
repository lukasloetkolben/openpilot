#include "tools/cabana/tools/findnewsignals.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QVBoxLayout>
#include <QtConcurrent>
#include <QInputDialog>
#include <QMessageBox>
#include <QGroupBox>

#include "tools/cabana/streams/abstractstream.h"

FindNewSignalsDlg::FindNewSignalsDlg(QWidget *parent) : QDialog(parent, Qt::WindowFlags() | Qt::Window) {
  setWindowTitle(tr("Find New Signals"));
  setAttribute(Qt::WA_DeleteOnClose);

  QVBoxLayout *main_layout = new QVBoxLayout(this);

  // Time range inputs
  QGroupBox *search_group = new QGroupBox(tr("Search Parameters"), this);
  QFormLayout *form_layout = new QFormLayout(search_group);
  start_time_edit = new QLineEdit("0", this);
  end_time_edit = new QLineEdit("10", this);
  start_time_edit->setValidator(new QDoubleValidator(0, 1000000, 3, this));
  end_time_edit->setValidator(new QDoubleValidator(0, 1000000, 3, this));

  form_layout->addRow(tr("Start Time (seconds):"), start_time_edit);
  form_layout->addRow(tr("End Time (seconds):"), end_time_edit);
  
  // Add bus filter input
  bus_filter_edit = new QLineEdit("", this);
  bus_filter_edit->setPlaceholderText(tr("e.g., 1,4,5 (empty for all buses)"));
  form_layout->addRow(tr("Filter Buses:"), bus_filter_edit);
  
  // Add minimum and maximum count filters
  QHBoxLayout *count_filter_layout = new QHBoxLayout();
  
  min_count_edit = new QLineEdit("0", this);
  min_count_edit->setPlaceholderText(tr("Min"));
  min_count_edit->setValidator(new QIntValidator(0, 1000000, this));
  min_count_edit->setMaximumWidth(100);
  
  max_count_edit = new QLineEdit("0", this);
  max_count_edit->setPlaceholderText(tr("Max (0=no limit)"));
  max_count_edit->setValidator(new QIntValidator(0, 1000000, this));
  max_count_edit->setMaximumWidth(120);
  
  count_filter_layout->addWidget(new QLabel(tr("Unique Values:")));
  count_filter_layout->addWidget(min_count_edit);
  count_filter_layout->addWidget(new QLabel("-"));
  count_filter_layout->addWidget(max_count_edit);
  count_filter_layout->addStretch();
  
  form_layout->addRow(count_filter_layout);

  // Filter options
  QHBoxLayout *filter_layout = new QHBoxLayout();
  filter_checkbox = new QCheckBox(tr("Filter by previous search:"), this);
  filter_combo = new QComboBox(this);
  filter_combo->setEnabled(false);
  filter_layout->addWidget(filter_checkbox);
  filter_layout->addWidget(filter_combo);
  filter_layout->addStretch();
  form_layout->addRow("", filter_layout);

  // Search and action buttons
  QHBoxLayout *button_layout = new QHBoxLayout();
  search_btn = new QPushButton(tr("Find New Signals"), this);
  copy_btn = new QPushButton(tr("Copy Selected IDs"), this);
  copy_btn->setEnabled(false);
  button_layout->addWidget(search_btn);
  button_layout->addWidget(copy_btn);
  button_layout->addStretch();

  // Results table
  table = new QTableWidget(this);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::ExtendedSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->horizontalHeader()->setStretchLastSection(true);
  table->setColumnCount(3);
  table->setHorizontalHeaderLabels({"Bus", "Message ID", "New Values Count"});

  // Saved searches section
  QGroupBox *saved_group = new QGroupBox(tr("Saved Searches"), this);
  QVBoxLayout *saved_layout = new QVBoxLayout(saved_group);
  saved_searches = new QListWidget(this);

  QHBoxLayout *saved_button_layout = new QHBoxLayout();
  save_search_btn = new QPushButton(tr("Save Current Search"), this);
  save_search_btn->setEnabled(false);
  clear_saved_btn = new QPushButton(tr("Clear Saved"), this);
  clear_saved_btn->setEnabled(false);

  saved_button_layout->addWidget(save_search_btn);
  saved_button_layout->addWidget(clear_saved_btn);
  saved_button_layout->addStretch();

  saved_layout->addWidget(saved_searches);
  saved_layout->addLayout(saved_button_layout);

  // Add all layouts to main layout
  main_layout->addWidget(search_group);
  main_layout->addLayout(button_layout);
  main_layout->addWidget(table);
  main_layout->addWidget(saved_group);

  setMinimumSize({750, 600});

  // Connect signals/slots
  QObject::connect(search_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::findNewSignals);
  QObject::connect(copy_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::copySelectedMessages);
  QObject::connect(save_search_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::saveCurrentSearch);
  QObject::connect(clear_saved_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::clearSavedMessages);
  QObject::connect(filter_checkbox, &QCheckBox::toggled, this, &FindNewSignalsDlg::toggleFilterMode);

  QObject::connect(table, &QTableWidget::itemSelectionChanged, [this]() {
    copy_btn->setEnabled(table->selectedItems().count() > 0);
  });

  QObject::connect(table, &QTableWidget::doubleClicked, [this](const QModelIndex &index) {
    if (index.isValid()) {
      bool ok;
      uint32_t address = table->item(index.row(), 1)->text().toUInt(&ok, 16);
      uint8_t bus = table->item(index.row(), 0)->text().toUInt();
      if (ok) {
        MessageId msg_id = {.source = bus, .address = address};
        emit openMessage(msg_id);
      }
    }
  });
}

void FindNewSignalsDlg::toggleFilterMode(bool checked) {
  filter_combo->setEnabled(checked);
}

void FindNewSignalsDlg::findNewSignals() {
  bool use_filter = filter_checkbox->isChecked();
  int filter_index = filter_combo->currentIndex();

  QSet<MessageIdentifier> filter_ids;
  if (use_filter && filter_index >= 0 && filter_index < saved_message_sets.size()) {
    filter_ids = saved_message_sets[filter_index];
  }

  findNewSignalsInternal(filter_ids, use_filter);

  // Enable save button after search is complete
  save_search_btn->setEnabled(true);
}

void FindNewSignalsDlg::findNewSignalsInternal(const QSet<MessageIdentifier> &filter_ids, bool use_filter) {
  search_btn->setEnabled(false);
  search_btn->setText(tr("Searching..."));

  // Convert time to mono_time
  double start_sec = start_time_edit->text().toDouble();
  double end_sec = end_time_edit->text().toDouble();
  uint64_t start_mono = can->toMonoTime(start_sec);
  uint64_t end_mono = can->toMonoTime(end_sec);
  uint64_t after_end_mono = can->toMonoTime(end_sec + 2.0); // 2 seconds after end

  // Set to store all message values seen during the start-end time range
  QSet<MessageValue> seen_messages;

  // Map to count new message values after the end time by message ID
  QHash<QPair<uint32_t, uint8_t>, int> new_messages_count;
  // Map to count unique values per message
  QHash<QPair<uint32_t, uint8_t>, QSet<QByteArray>> unique_values_per_message;

  // Process all events
  const auto &events = can->allEvents();

  // Parse bus filter
  QSet<uint8_t> bus_filter;
  QString bus_filter_text = bus_filter_edit->text().trimmed();
  if (!bus_filter_text.isEmpty()) {
    QStringList bus_list = bus_filter_text.split(',');
    for (const QString &bus_str : bus_list) {
      bool ok;
      int bus = bus_str.trimmed().toInt(&ok);
      if (ok && bus >= 0 && bus <= 0xFF) {
        bus_filter.insert(static_cast<uint8_t>(bus));
      }
    }
  }

  // First phase: collect all message values in the specified time range
  for (const CanEvent *e : events) {
    // Skip if bus filter is active and this message's bus is not in the filter
    if (!bus_filter.isEmpty() && !bus_filter.contains(e->src)) {
      continue;
    }
    
    // Skip if we're using a filter and this message ID is not in our filter set
    if (use_filter) {
      MessageIdentifier msg_id = {.address = e->address, .bus = e->src};
      if (!filter_ids.contains(msg_id)) {
        continue;
      }
    }

    if (e->mono_time >= start_mono && e->mono_time <= end_mono) {
      MessageValue mv = {
        .address = e->address,
        .bus = e->src,
        .data = QByteArray((const char*)e->dat, e->size)
      };
      seen_messages.insert(mv);
    }
  }

  // Second phase: check for new message values after the specified end time
  for (const CanEvent *e : events) {
    // Skip if bus filter is active and this message's bus is not in the filter
    if (!bus_filter.isEmpty() && !bus_filter.contains(e->src)) {
      continue;
    }
    
    // Skip if we're using a filter and this message ID is not in our filter set
    if (use_filter) {
      MessageIdentifier msg_id = {.address = e->address, .bus = e->src};
      if (!filter_ids.contains(msg_id)) {
        continue;
      }
    }

    if (e->mono_time > end_mono && e->mono_time <= after_end_mono) {
      MessageValue mv = {
        .address = e->address,
        .bus = e->src,
        .data = QByteArray((const char*)e->dat, e->size)
      };

      auto key = qMakePair(e->address, e->src);
      // If this message value wasn't seen in the initial time range
      if (!seen_messages.contains(mv)) {
        new_messages_count[key]++;
        // Add to seen messages to avoid counting it multiple times
        seen_messages.insert(mv);
      }
      // Track unique values for this message
      unique_values_per_message[key].insert(QByteArray((const char*)e->dat, e->size));
    }
  }

  // Get min/max unique values thresholds
  int min_unique_values = min_count_edit->text().toInt();
  int max_unique_values = max_count_edit->text().toInt();
  
  // Count messages that pass the filters
  int filtered_count = 0;
  for (auto it = new_messages_count.begin(); it != new_messages_count.end(); ++it) {
    int unique_count = unique_values_per_message[it.key()].size();
    if (unique_count >= min_unique_values && 
        (max_unique_values == 0 || unique_count <= max_unique_values)) {
      filtered_count++;
    }
  }
  
  // Display results in the table
  table->setRowCount(filtered_count);
  int row = 0;

  for (auto it = new_messages_count.begin(); it != new_messages_count.end(); ++it) {
    const auto &key = it.key();
    int unique_value_count = unique_values_per_message[key].size();
    
    // Skip messages that don't meet the unique values thresholds
    if (unique_value_count < min_unique_values || 
        (max_unique_values > 0 && unique_value_count > max_unique_values)) {
      continue;
    }
    
    int new_messages = it.value();
    
    table->setItem(row, 0, new QTableWidgetItem(QString::number(key.second)));
    table->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(key.first, 1, 16)));
    table->setItem(row, 2, new QTableWidgetItem(QString("%1 (unique: %2)")
      .arg(new_messages)
      .arg(unique_value_count)));
    
    row++;
  }

  // Sort by count descending
  table->sortByColumn(2, Qt::DescendingOrder);

  search_btn->setEnabled(true);
  search_btn->setText(tr("Find New Signals"));
}

void FindNewSignalsDlg::copySelectedMessages() {
  // Collect all selected message IDs
  QSet<MessageIdentifier> selected_ids;

  QList<QTableWidgetItem*> selected_items = table->selectedItems();
  QSet<int> selected_rows;

  // Get unique rows that are selected
  for (QTableWidgetItem* item : selected_items) {
    selected_rows.insert(item->row());
  }

  // For each selected row, get the message ID
  for (int row : selected_rows) {
    bool ok;
    uint8_t bus = table->item(row, 0)->text().toUInt();
    uint32_t address = table->item(row, 1)->text().toUInt(&ok, 16);

    if (ok) {
      MessageIdentifier msg_id = {.address = address, .bus = bus};
      selected_ids.insert(msg_id);
    }
  }

  if (selected_ids.isEmpty()) {
    QMessageBox::information(this, tr("No Selection"), tr("No message IDs selected."));
    return;
  }

  // Add selected messages to saved list
  bool ok;
  QString search_name = QInputDialog::getText(this, tr("Name This Selection"),
                                             tr("Enter a name for this selection:"), QLineEdit::Normal,
                                             tr("Search %1").arg(saved_message_sets.size() + 1), &ok);

  if (ok && !search_name.isEmpty()) {
    saved_message_sets.append(selected_ids);
    saved_search_names.append(search_name);
    saved_searches->addItem(search_name);
    filter_combo->addItem(search_name);

    QMessageBox::information(this, tr("Selection Saved"),
                           tr("%1 message IDs saved as \"%2\"")
                           .arg(selected_ids.size())
                           .arg(search_name));

    clear_saved_btn->setEnabled(true);
  }
}

void FindNewSignalsDlg::saveCurrentSearch() {
  // Collect all message IDs from the current search
  QSet<MessageIdentifier> current_ids;

  for (int row = 0; row < table->rowCount(); row++) {
    bool ok;
    uint8_t bus = table->item(row, 0)->text().toUInt();
    uint32_t address = table->item(row, 1)->text().toUInt(&ok, 16);

    if (ok) {
      MessageIdentifier msg_id = {.address = address, .bus = bus};
      current_ids.insert(msg_id);
    }
  }

  if (current_ids.isEmpty()) {
    QMessageBox::information(this, tr("Empty Search"), tr("No message IDs in current search results."));
    return;
  }

  // Add current search results to saved list
  bool ok;
  QString search_name = QInputDialog::getText(this, tr("Name This Search"),
                                             tr("Enter a name for this search:"), QLineEdit::Normal,
                                             tr("Search %1").arg(saved_message_sets.size() + 1), &ok);

  if (ok && !search_name.isEmpty()) {
    saved_message_sets.append(current_ids);
    saved_search_names.append(search_name);
    saved_searches->addItem(search_name);
    filter_combo->addItem(search_name);

    QMessageBox::information(this, tr("Search Saved"),
                           tr("%1 message IDs saved as \"%2\"")
                           .arg(current_ids.size())
                           .arg(search_name));

    clear_saved_btn->setEnabled(true);
  }
}

void FindNewSignalsDlg::clearSavedMessages() {
  if (saved_message_sets.isEmpty()) {
    return;
  }

  QMessageBox::StandardButton reply = QMessageBox::question(this, tr("Clear Saved Searches"),
                                    tr("Are you sure you want to clear all saved searches?"),
                                    QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes) {
    saved_message_sets.clear();
    saved_search_names.clear();
    saved_searches->clear();
    filter_combo->clear();

    filter_checkbox->setChecked(false);
    filter_combo->setEnabled(false);
    clear_saved_btn->setEnabled(false);
  }
}
