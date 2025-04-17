#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>
#include <QHBoxLayout>
#include <QLabel>
#include <QStyleOption>

#include "selfdrive/ui/qt/util.h"
#include "common/swaglog.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent), params(Params()) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["selfdriveState"].getSelfdriveState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  QPixmap img = experimental_mode ? experimental_img : engage_img;
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0);
}

// SpeedControl
SpeedControl::SpeedControl(QWidget *parent) : QWidget(parent), speed(30) {
  pm = std::make_unique<PubMaster>(std::vector<const char*>{"uiSetValues"});

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  QFont buttonFont;
  buttonFont.setPointSize(12);
  QPalette buttonPalette;
  buttonPalette.setColor(QPalette::ButtonText, Qt::white);
  QString buttonStyleSheet = "QPushButton {"
                             "border: 15px solid #151515;"
                             "border-radius: 30px;"
                             "background-color: rgba(21, 21, 21, 0.4);"
                             "font-size: 240px;"
                             "}"
                             "QPushButton:pressed {"
                             "background-color: rgba(84, 84, 84, 0.7);"
                             "}";

  decreaseButton = new QPushButton("-", this);
  decreaseButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  decreaseButton->setFont(buttonFont);
  decreaseButton->setPalette(buttonPalette);
  decreaseButton->setStyleSheet(buttonStyleSheet);
  layout->addWidget(decreaseButton);

  increaseButton = new QPushButton("+", this);
  increaseButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  increaseButton->setFont(buttonFont);
  increaseButton->setPalette(buttonPalette);
  increaseButton->setStyleSheet(buttonStyleSheet);
  layout->addWidget(increaseButton);

  QObject::connect(increaseButton, &QPushButton::clicked, [this]() {
    speed = std::clamp<int>(speed + 5, 5, 90);
    setSpeedValue = QString::number(speed);

    MessageBuilder msg;
    auto m = msg.initEvent().initUiSetValues();
    m.setUiSetSpeed(speed);
    pm->send("uiSetValues", msg);
  });

  QObject::connect(decreaseButton, &QPushButton::clicked, [this]() {
    speed = std::clamp<int>(speed - 5, 5, 90);
    setSpeedValue = QString::number(speed);
    MessageBuilder msg;
    auto m = msg.initEvent().initUiSetValues();
    m.setUiSetSpeed(speed);
    pm->send("uiSetValues", msg);
  });

  setLayout(layout);
}
