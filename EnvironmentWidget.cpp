//<<<<<<< Temporary merge branch 1
#include "EnvironmentWidget.h"
#include <QtGui>

const static qreal w = 330.0;
const static qreal h = 240.0-24;
const static QColor bgColor(50, 76, 90);
const static int refreshTime = 30;
const static int maxWindPower = 20;

EnvironmentWidget::EnvironmentWidget(QWidget *parent) : QWidget(parent){
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(refreshTime);
    setWindowTitle(tr("Environment"));
    setMinimumSize(300, 200);
    temp = new TPHChart(-40, 60);
    pressure = new TPHChart(30, 110);
    humidity = new TPHChart(0, 100);
    wind = new WindChart(maxWindPower);
    resize(w, h);
}

void EnvironmentWidget::paintEvent(QPaintEvent *) {
    float coefficient = 1; //1.3

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // set scale and translate param
    int dx = 0;
    float   sx = width() / w,
            sy = height() / h,
            rsx = 1;
    // Background
    painter.fillRect(QRect(0, 0, w * sx, h * sy), QBrush(bgColor));
    if (sx > sy) {
        dx = (width() - w * sy) / sy / 5;
        painter.scale(sy, sy);
        rsx = sy;
    } else {
        if (sy / sx < coefficient) {
            painter.scale(sx, sy);
        } else {
            painter.translate(0, (height() - h * sx * coefficient) / 2);
            painter.scale(sx, sx * coefficient);
        }
        rsx = sx;
    }
    // Background
    painter.fillRect(QRect(0, 0, w, h), QBrush(bgColor));

    painter.save();
        painter.translate(10 + dx, 45-24);
        temp->draw(painter);
    painter.restore();
    painter.save();
        painter.translate(68 + 2 * dx, 45-24);
        pressure->draw(painter);
    painter.restore();
    painter.save();
        painter.translate(125 + 3 * dx, 45-24);
        humidity->draw(painter);
    painter.restore();
    painter.save();
        painter.translate(180 + 4 * dx, 40-24);
        wind->draw(painter);
    painter.restore();

    // draw text
    QString str;
    QFont font;
    font.setBold(true);
    painter.setPen(QPen(Qt::white, 1));
    font.setPixelSize(12);
    painter.setFont(font);
    painter.drawText(QRect( 15 + dx, 25-24,  55, 14), Qt::AlignCenter, "T");
    painter.drawText(QRect( 73 + 2 * dx, 25-24,  55, 14), Qt::AlignCenter, "P");
    painter.drawText(QRect(130 + 3 * dx, 25-24,  55, 14), Qt::AlignCenter, "RH");
    painter.drawText(QRect(180 + 4 * dx, 25-24, 140, 14), Qt::AlignCenter, "Wind");
    font.setPixelSize(12);
    painter.setFont(font);

    str.setNum(this->wind->getPower());
    painter.drawText(QRect(180 + 4 * dx, 207-24, 150, 14), Qt::AlignCenter, "Vertical = " + str);
    str.setNum(wind->getTotalSpeed(), 'q', 3);
    painter.drawText(QRect(180 + 4 * dx, 220-24,  150, 14), Qt::AlignCenter, "Total = " + str + " m/s");

    // draw wing picker
    QLinearGradient wingPicker(180 + 4 * dx, 0, 320 + 4 * dx, 0);
    wingPicker.setColorAt(0.0, Qt::yellow);
    wingPicker.setColorAt(1.0, Qt::red);
    painter.fillRect(QRect(179 + 4 * dx, 185-24, 142, 8), Qt::white);
    painter.fillRect(QRect(180 + 4 * dx, 186-24, 140, 6), wingPicker);
    font.setPixelSize(10);
    painter.setFont(font);
    for (int i = 0; i < 6; i++) {
        str.setNum(maxWindPower / 5 * i);
        if  (i == 5) str.setNum(maxWindPower);
        painter.drawText(QRect(4 * dx + 175 + (129 / 5 * i), 195-24,  20, 12), Qt::AlignCenter, str);
    }

}

void EnvironmentWidget::setTemp(float temp){
    this->temp->setValue(temp);
}

void EnvironmentWidget::setPressure(float pressure){
    this->pressure->setValue(pressure);
}

void EnvironmentWidget::setHumidity(float humidity){
    this->humidity->setValue(humidity);
}

void EnvironmentWidget::setWind(float alpha, float betta, float power){
    this->wind->setValue(alpha, betta, power);
}
