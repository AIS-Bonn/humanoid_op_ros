// Provides access to camera settings (e.g. white balance)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "cameraparamswidget.h"

#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/qlayout.h>
#include <boost/make_shared.hpp>

CameraParamsWidget::CameraParamsWidget(QWidget* parent)
 : QScrollArea(parent)
{
	m_w = new QWidget(this);
	setWidget(m_w);
	setWidgetResizable(true);

	QGridLayout* layout = new QGridLayout(m_w);
	layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
	m_w->setLayout(layout);
}

CameraParamsWidget::~CameraParamsWidget()
{
}

QWidget* CameraParamsWidget::createEditorFor(const camera_v4l2::CameraParam& param)
{
	if(!param.choices.empty())
	{
		QComboBox* box = new QComboBox(m_w);
		for(uint i = 0; i < param.choices.size(); ++i)
		{
			box->addItem(
				param.choices[i].label.c_str(),
				param.choices[i].value
			);
			if(param.choices[i].value == param.value)
				box->setCurrentIndex(i);
		}

		box->setProperty("paramID", param.id);

		connect(box, SIGNAL(activated(int)), SLOT(updateComboBox()));

		return box;
	}

	if(param.minimum == 0 && param.maximum == 1)
	{
		QCheckBox* box = new QCheckBox(m_w);
		box->setChecked(param.value);

		box->setProperty("paramID", param.id);

		connect(box, SIGNAL(clicked(bool)), SLOT(updateCheckBox(bool)));

		return box;
	}

	QSlider* slider = new QSlider(Qt::Horizontal, m_w);
	slider->setMinimum(param.minimum);
	slider->setMaximum(param.maximum);
	slider->setValue(param.value);

	slider->setProperty("paramID", param.id);

	connect(slider, SIGNAL(valueChanged(int)), SLOT(updateSlider(int)));

	return slider;
}

void CameraParamsWidget::setParamInfo(const camera_v4l2::EnumerateCameraParamsResponse::Ptr& resp)
{
	m_paramInfo = resp;
	QGridLayout* l = (QGridLayout*)m_w->layout();

	for(uint i = 0; i < resp->params.size(); ++i)
	{
		const camera_v4l2::CameraParam& param = resp->params[i];

		QLabel* label = new QLabel(param.label.c_str(), m_w);
		l->addWidget(label, i, 0);

		QWidget* editor = createEditorFor(param);
		l->addWidget(editor, i, 1);
	}
}

void CameraParamsWidget::updateCheckBox(bool checked)
{
	QWidget* s = (QWidget*)sender();

	camera_v4l2::SetCameraParamRequest::Ptr req =
		boost::make_shared<camera_v4l2::SetCameraParamRequest>();
	req->param.id = s->property("paramID").toInt();
	req->param.value = checked ? 1 : 0;

	emit setParam(req);
}

void CameraParamsWidget::updateComboBox()
{
	QComboBox* s = (QComboBox*)sender();

	camera_v4l2::SetCameraParamRequest::Ptr req =
		boost::make_shared<camera_v4l2::SetCameraParamRequest>();
	req->param.id = s->property("paramID").toInt();
	req->param.value = s->itemData(s->currentIndex()).toInt();

	emit setParam(req);
}

void CameraParamsWidget::updateSlider(int value)
{
	QWidget* s = (QWidget*)sender();

	camera_v4l2::SetCameraParamRequest::Ptr req =
		boost::make_shared<camera_v4l2::SetCameraParamRequest>();
	req->param.id = s->property("paramID").toInt();
	req->param.value = value;

	emit setParam(req);
}

