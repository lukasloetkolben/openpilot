#pragma once

#include <QDialog>
#include <QHash>
#include <QSet>
#include <QPair>
#include <QTableWidget>
#include <QLineEdit>
#include <QPushButton>

#include "tools/cabana/dbc/dbcmanager.h"

class FindNewSignalsDlg : public QDialog {
  Q_OBJECT

public:
  FindNewSignalsDlg(QWidget *parent);

  // Structure to represent a message with its value
  struct MessageValue {
    uint32_t address;
    uint8_t bus;
    QByteArray data;  // Full message data

    bool operator==(const MessageValue &other) const {
      return address == other.address &&
             bus == other.bus &&
             data == other.data;
    }
  };

signals:
  void openMessage(const MessageId &msg_id);

private:
  void findNewSignals();

  QTableWidget *table;
  QLineEdit *start_time_edit, *end_time_edit;
  QPushButton *search_btn;
};

// Define Qt hash function for MessageValue
inline uint qHash(const FindNewSignalsDlg::MessageValue &key, uint seed = 0) {
  return qHash(key.address, seed) ^
         qHash(key.bus, seed) ^
         qHash(key.data, seed);
}
