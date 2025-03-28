#include "tools/cabana/tools/findnewsignals.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QVBoxLayout>
#include <QtConcurrent>

#include "tools/cabana/streams/abstractstream.h"

FindNewSignalsDlg::FindNewSignalsDlg(QWidget *parent) : QDialog(parent, Qt::WindowFlags() | Qt::Window) {
  setWindowTitle(tr("Find New Signals"));
  setAttribute(Qt::WA_DeleteOnClose);

  QVBoxLayout *main_layout = new QVBoxLayout(this);

  // Time range inputs
  QFormLayout *form_layout = new QFormLayout();
  start_time_edit = new QLineEdit("0", this);
  end_time_edit = new QLineEdit("10", this);
  start_time_edit->setValidator(new QDoubleValidator(0, 1000000, 3, this));
  end_time_edit->setValidator(new QDoubleValidator(0, 1000000, 3, this));

  form_layout->addRow(tr("Start Time (seconds):"), start_time_edit);
  form_layout->addRow(tr("End Time (seconds):"), end_time_edit);

  // Search button
  QHBoxLayout *button_layout = new QHBoxLayout();
  search_btn = new QPushButton(tr("Find New Signals"), this);
  button_layout->addStretch();
  button_layout->addWidget(search_btn);

  // Results table
  table = new QTableWidget(this);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->horizontalHeader()->setStretchLastSection(true);
  table->setColumnCount(3);
  table->setHorizontalHeaderLabels({"Bus", "Message ID", "New Values Count"});

  main_layout->addLayout(form_layout);
  main_layout->addLayout(button_layout);
  main_layout->addWidget(table);

  setMinimumSize({600, 400});

  // Connect signals/slots
  QObject::connect(search_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::findNewSignals);
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

void FindNewSignalsDlg::findNewSignals() {
  search_btn->setEnabled(false);
  search_btn->setText(tr("Searching..."));

  // Convert time to mono_time
  double start_sec = start_time_edit->text().toDouble();
  double end_sec = end_time_edit->text().toDouble();
  uint64_t start_mono = can->toMonoTime(start_sec);
  uint64_t end_mono = can->toMonoTime(end_sec);
  uint64_t after_end_mono = can->toMonoTime(end_sec + 3.0); // 3 seconds after end

  // Set to store all message values seen during the start-end time range
  QSet<MessageValue> seen_messages;

  // Map to count new message values after the end time by message ID
  QHash<QPair<uint32_t, uint8_t>, int> new_messages_count;

  // Process all events
  const auto &events = can->allEvents();

  // First phase: collect all message values in the specified time range
  for (const CanEvent *e : events) {
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
    if (e->mono_time > end_mono && e->mono_time <= after_end_mono) {
      MessageValue mv = {
        .address = e->address,
        .bus = e->src,
        .data = QByteArray((const char*)e->dat, e->size)
      };

      // If this message value wasn't seen in the initial time range
      if (!seen_messages.contains(mv)) {
        auto key = qMakePair(e->address, e->src);
        new_messages_count[key]++;

        // Add to seen messages to avoid counting it multiple times
        seen_messages.insert(mv);
      }
    }
  }

  // Display results in the table
  table->setRowCount(new_messages_count.size());
  int row = 0;

  for (auto it = new_messages_count.begin(); it != new_messages_count.end(); ++it) {
    const auto &key = it.key();
    int count = it.value();

    table->setItem(row, 0, new QTableWidgetItem(QString::number(key.second)));
    table->setItem(row, 1, new QTableWidgetItem(QString("%1").arg(key.first, 1, 16)));
    table->setItem(row, 2, new QTableWidgetItem(QString::number(count)));

    row++;
  }

  // Sort by count descending
  table->sortByColumn(2, Qt::DescendingOrder);

  search_btn->setEnabled(true);
  search_btn->setText(tr("Find New Signals"));
}
