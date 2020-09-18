#include "A1_2Dwidget.h"

#include <QMouseEvent>

#include <math.h>

#include <iostream>


#include "../layers/A1_2dlayer.h"

using namespace std;



A1_2DWidget::A1_2DWidget(QWidget *parent) :
    OGLTWidget(parent)
{

   m_layers.push_back(new A1_2DLayer(this));

}

A1_2DWidget::~A1_2DWidget()
{

}

void A1_2DWidget::keyPressEvent(QKeyEvent *event){


    OGLTWidget::keyPressEvent(event);
}


void A1_2DWidget::keyReleaseEvent(QKeyEvent *event){


    OGLTWidget::keyReleaseEvent(event);
}


