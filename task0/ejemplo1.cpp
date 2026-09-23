#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button1, SIGNAL(clicked()), this, SLOT(doButton()) );	
	connect(button1, SIGNAL(clicked()), this, SLOT(detenerTemporizador()));
	connect(button_2, SIGNAL(clicked()), this, SLOT(reiniciar()));
	connect(button_3, SIGNAL(clicked()), this, SLOT(cambiarContador()));
	connect(&timer, SIGNAL(timeout()), this, SLOT(avanzarTemporizador()));
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(modificarVelocidad(int)));
	// BOJETO QUE EMITE LA SEÑAL,signal, OBJETO Q TIENE EL SLOT, SLOT dnd quieres enviar la señal.
	timer.start(200);
}
void ejemplo1::doButton()
{
	qDebug() << "click on button";
}

void ejemplo1 :: avanzarTemporizador()
{
	int v = lcdNumber->value();

	if(button_3->text() == "COUNTDOWN"){
		lcdNumber->display(++v);
	}else{
		lcdNumber->display(--v);
	}
    
}

void ejemplo1 :: detenerTemporizador () {

	if(button1->text() == "STOP"){
		timer.stop();
		button1->setText("START");
	}
	else{
		timer.start();
		button1->setText("STOP");
	}
}

void ejemplo1 :: reiniciar()
{
	int v = lcdNumber->value();
	v = 0;
    lcdNumber->display(v);
}

void ejemplo1 :: modificarVelocidad(int velocidad){

	lcdNumber_2->display(velocidad);
	timer.start(velocidad);
}

void ejemplo1 :: cambiarContador(){

	if(button_3->text() == "COUNTDOWN"){
		button_3->setText("COUNTUP");
	}else{
		button_3->setText("COUNTDOWN");
	}

}




