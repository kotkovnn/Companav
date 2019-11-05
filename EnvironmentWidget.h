#ifndef ENVIRONMENTWIDGET_H
#define ENVIRONMENTWIDGET_H

#include <QWidget>
#include "TPHChart.h"
#include "WindChart.h"
#include <QtGui>

class EnvironmentWidget : public QWidget {
    Q_OBJECT

    private: TPHChart *temp, *pressure, *humidity;
    private: WindChart *wind;

    public slots: void setTemp(float temp);
    public slots: void setPressure(float pressure);
    public slots: void setHumidity(float humidity);
    public slots: void setWind(float alpha, float betta, float power);

    public: explicit EnvironmentWidget(QWidget *parent = 0);
    protected: void paintEvent(QPaintEvent *event);
       
};

#endif // ENVIRONMENTWIDGET_H
