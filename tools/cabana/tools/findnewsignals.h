#ifndef FINDNEWSIGNALS_H
#define FINDNEWSIGNALS_H

#include <QDialog>
#include <QLineEdit>
#include <QTableWidget>
#include <QPushButton>
#include "tools/cabana/streams/abstractstream.h"
#include "tools/cabana/dbc/dbcmanager.h"

struct new_signal_struct {
  uint32_t address;
  QString data;
  QString time_seen;
};

class FindNewSignalsDlg : public QDialog {
  Q_OBJECT

public:
  explicit FindNewSignalsDlg(QWidget *parent = nullptr);

private slots:
  void findNewSignals();

signals:
  void openMessage(const MessageId &msg_id);

private:
  QLineEdit *end_time_edit;
  QLineEdit *blacklist_edit;
  QTableWidget *table;
  QPushButton *search_btn;
};

#endif // FINDNEWSIGNALS_H
