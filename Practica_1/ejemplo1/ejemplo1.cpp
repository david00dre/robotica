#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
    num = 0;
    bot =false;
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(funcionTimer()));
}

void ejemplo1::doButton()
{
    bot = !bot;
    bot ? timer->start(250) : timer->stop();
}

void ejemplo1::funcionTimer()
{
    lcdNumber->display(++num);
}




