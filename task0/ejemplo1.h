#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>

class ejemplo1 : public QWidget, public Ui_Counter
{

    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doButton();
        void avanzarTemporizador();
        void detenerTemporizador();
        void reiniciar();
        void modificarVelocidad(int velocidad);
        void cambiarContador();

    private:
        QTimer timer;
};

#endif // ejemplo1_H