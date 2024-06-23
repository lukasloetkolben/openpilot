#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QSet>
#include <iostream>
#include <vector>
#include <cstring>
#include <QMap>
#include <QVBoxLayout>
#include "tools/cabana/tools/findnewsignals.h"
#include "tools/cabana/dbc/dbcmanager.h"

FindNewSignalsDlg::FindNewSignalsDlg(QWidget *parent) : QDialog(parent, Qt::WindowFlags() | Qt::Window) {
  setWindowTitle(tr("Find New Signal"));
  setAttribute(Qt::WA_DeleteOnClose);

  QVBoxLayout *main_layout = new QVBoxLayout(this);

  QHBoxLayout *timestamp_layout = new QHBoxLayout();
  end_time_edit = new QLineEdit(this);
  end_time_edit->setPlaceholderText("Time in seconds");

  search_btn = new QPushButton(tr("&Search"), this);

  timestamp_layout->addWidget(new QLabel(tr("End time")));
  timestamp_layout->addWidget(end_time_edit);
  timestamp_layout->addWidget(search_btn);

  main_layout->addLayout(timestamp_layout);

  QHBoxLayout *blacklist_layout = new QHBoxLayout();
  blacklist_edit = new QLineEdit(this);
  blacklist_edit->setPlaceholderText("Comma separated addresses to ignore");

  blacklist_layout->addWidget(new QLabel(tr("Blacklist")));
  blacklist_layout->addWidget(blacklist_edit);

  main_layout->addLayout(blacklist_layout);

  table = new QTableWidget(this);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->horizontalHeader()->setStretchLastSection(true);
  table->setSortingEnabled(true);  // Enable sorting
  main_layout->addWidget(table);

  setMinimumSize({700, 500});
  connect(search_btn, &QPushButton::clicked, this, &FindNewSignalsDlg::findNewSignals);
  connect(table->horizontalHeader(), &QHeaderView::sectionClicked, table, &QTableWidget::sortByColumn);  // Connect header click to sortByColumn
}

void FindNewSignalsDlg::findNewSignals() {
  qint64 target_time = end_time_edit->text().toInt();
  const auto &events = can->allEvents();
  QMap<uint32_t, int> address_counts;
  QSet<QString> messages;
  QStringList blacklist_list = blacklist_edit->text().split(",", QString::SkipEmptyParts);
  QSet<uint32_t> blacklist;

  for (const QString &address : blacklist_list) {
    blacklist.insert(address.trimmed().toUInt(nullptr, 16));
  }

  for (const CanEvent *e : events) {
    qint64 event_time = e->mono_time / 1e9 - can->routeStartTime();
    QString data_vec = QString::number(e->address) + QString(QByteArray::fromRawData(reinterpret_cast<const char*>(e->dat), e->size));

    if (blacklist.contains(e->address)) {
      continue;
    }

    if (event_time < target_time) {
      messages.insert(data_vec);
    } else if (event_time < (target_time + 2) && !messages.contains(data_vec)) {
      address_counts[e->address]++;
      messages.insert(data_vec);
    }
  }

  table->clear();
  table->setRowCount(address_counts.size());
  table->setColumnCount(3);
  table->setHorizontalHeaderLabels({"Message Name", "Address", "Count"});

  int row = 0;
  for (auto it = address_counts.constBegin(); it != address_counts.constEnd(); ++it, ++row) {
    uint32_t address = it.key();
    table->setItem(row, 0, new QTableWidgetItem(msgName({0, address})));
    table->setItem(row, 1, new QTableWidgetItem(QString::number(address, 16)));
    table->setItem(row, 2, new QTableWidgetItem(QString::number(it.value())));
  }
}
