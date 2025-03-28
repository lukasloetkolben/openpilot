#pragma once

#include <QDialog>
#include <QHash>
#include <QSet>
#include <QPair>
#include <QTableWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QListWidget>

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

  // Structure to represent a message identifier (address + bus)
  struct MessageIdentifier {
    uint32_t address;
    uint8_t bus;

    bool operator==(const MessageIdentifier &other) const {
      return address == other.address && bus == other.bus;
    }
  };

signals:
  void openMessage(const MessageId &msg_id);

private slots:
  void findNewSignals();
  void copySelectedMessages();
  void toggleFilterMode(bool checked);
  void clearSavedMessages();
  void saveCurrentSearch();

private:
  void findNewSignalsInternal(const QSet<MessageIdentifier> &filter_ids = {}, bool use_filter = false);

  QTableWidget *table;
  QLineEdit *start_time_edit, *end_time_edit;
  QPushButton *search_btn;
  QPushButton *copy_btn;
  QPushButton *save_search_btn;
  QPushButton *clear_saved_btn;
  QCheckBox *filter_checkbox;
  QListWidget *saved_searches;

  // Store message IDs from previous searches
  QList<QSet<MessageIdentifier>> saved_message_sets;
  QStringList saved_search_names;
  QComboBox *filter_combo;
};

// Define Qt hash function for MessageValue
inline uint qHash(const FindNewSignalsDlg::MessageValue &key, uint seed = 0) {
  return qHash(key.address, seed) ^
         qHash(key.bus, seed) ^
         qHash(key.data, seed);
}

// Define Qt hash function for MessageIdentifier
inline uint qHash(const FindNewSignalsDlg::MessageIdentifier &key, uint seed = 0) {
  return qHash(key.address, seed) ^ qHash(key.bus, seed);
}
