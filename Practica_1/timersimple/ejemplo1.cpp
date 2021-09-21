#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));


	mytimer.connect(std::bind(&ejemplo1::cuenta, this));
    mytimer.start(500);
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
    static bool stopped = false;
    stopped = !stopped;
	if(stopped) {
        button->setText("START");
        mytimer.stop();
    }
	else {
        button->setText("STOP");
        mytimer.start(mytimer.getPeriod());
    }

}

void ejemplo1::cuenta()
{
    lcdNumber->display(++cont);
	trick++;
}

