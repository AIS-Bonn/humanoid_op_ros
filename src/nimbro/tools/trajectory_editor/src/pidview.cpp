#include <trajectory_editor/spaces/pidview.h>

#include <math.h>

const float PIDView::p_ColorBoundary = 0.4;
const float PIDView::i_ColorBoundary = 0.15;
const float PIDView::d_ColorBoundary = 0.15;

PIDView::PIDView(BasicSmallView::Alignment alignment, BasicSmallView::Type type, std::string  jointName, bool shiftMirrored, QWidget *parent)
	: BasicSmallView(alignment, type, jointName, shiftMirrored, parent)
{
	// Init gui
	p_Spin = new QDoubleSpinBox();
	i_Spin = new QDoubleSpinBox();
	d_Spin = new QDoubleSpinBox();
	
	limit_Spin = new QDoubleSpinBox();
	
	p_Radio = new QRadioButton("R");
	i_Radio = new QRadioButton("P");
	d_Radio = new QRadioButton("Y");
	
	p_History = new HistoryKeeper(10, 1000, p_Spin);
	i_History = new HistoryKeeper(10, 1000, i_Spin);
	d_History = new HistoryKeeper(10, 1000, d_Spin);
	
	zero_sheet   = QString("QDoubleSpinBox { background: rgb(255, 255, 255); }");
	medium_sheet = QString("QDoubleSpinBox { background: rgb(110, 255, 90); }");
	large_sheet  = QString("QDoubleSpinBox { background: rgb(225, 185, 255); }");
	
	// Configure PID spins
	p_Spin->setMinimum(min);
	p_Spin->setMaximum(max);
	p_Spin->setValue(0);
	p_Spin->setSingleStep(0.01);
	p_Spin->setToolTip("p_Gain");
	
	i_Spin->setMinimum(min);
	i_Spin->setMaximum(max);
	i_Spin->setValue(0);
	i_Spin->setSingleStep(0.01);
	i_Spin->setToolTip("i_Gain");
	
	d_Spin->setMinimum(min);
	d_Spin->setMaximum(max);
	d_Spin->setValue(0);
	d_Spin->setSingleStep(0.01);
	d_Spin->setToolTip("d_Gain");
	
	// Configure limit
	limit_Spin->setMinimum(0);
	limit_Spin->setMaximum(M_PI);
	limit_Spin->setValue(0);
	limit_Spin->setSingleStep(0.01);
	limit_Spin->setToolTip("limit");
	
	// Configure radio buttons
	p_Radio->setAutoExclusive(false);
	i_Radio->setAutoExclusive(false);
	d_Radio->setAutoExclusive(false);
	
	// Set up widgets alignment and layout
	std::vector<QWidget*> widgets;
	widgets.push_back(p_Spin);
	widgets.push_back(i_Spin);
	widgets.push_back(d_Spin);
	
	widgets.push_back(limit_Spin);
	
	widgets.push_back(p_Radio);
	widgets.push_back(i_Radio);
	widgets.push_back(d_Radio);
	
	setUpLayout(widgets, alignment);
	
	// Set up connections
	connect(p_Spin, SIGNAL(valueChanged(double)), this, SLOT(p_SpinChanged()));
	connect(i_Spin, SIGNAL(valueChanged(double)), this, SLOT(i_SpinChanged()));
	connect(d_Spin, SIGNAL(valueChanged(double)), this, SLOT(d_SpinChanged()));
	connect(limit_Spin, SIGNAL(valueChanged(double)), this, SLOT(limitSpinChanged()));
	
	connect(p_Radio, SIGNAL(toggled(bool)), this, SLOT(p_RadioChanged()));
	connect(i_Radio, SIGNAL(toggled(bool)), this, SLOT(i_RadioChanged()));
	connect(d_Radio, SIGNAL(toggled(bool)), this, SLOT(d_RadioChanged()));
}

void PIDView::clearHistoryOfChanges()
{
	p_History->clearHistory();
	i_History->clearHistory();
	d_History->clearHistory();
}

void PIDView::setField(Field field, double value)
{
	if(field == PIDView::P_VALUE)
	{
		p_Spin->blockSignals(true);
		p_Spin->setValue(value);
		updateBackgroundColor(p_Spin, p_ColorBoundary);
		p_Spin->blockSignals(false);
	}
	else if(field == PIDView::I_VALUE)
	{
		i_Spin->blockSignals(true);
		i_Spin->setValue(value);
		updateBackgroundColor(i_Spin, i_ColorBoundary);
		i_Spin->blockSignals(false);
	}
	else if(field == PIDView::D_VALUE)
	{
		d_Spin->blockSignals(true);
		d_Spin->setValue(value);
		updateBackgroundColor(d_Spin, d_ColorBoundary);
		d_Spin->blockSignals(false);
	}
	else if(field == PIDView::LIMIT)
	{
		limit_Spin->blockSignals(true);
		limit_Spin->setValue(value);
		updateBackgroundColor(limit_Spin, p_ColorBoundary);
		limit_Spin->blockSignals(false);
	}
	
	else if(field == PIDView::PID_FLAG)
	{
		blockRadioButtons(true);
		
		p_Radio->setChecked(false);
		i_Radio->setChecked(false);
		d_Radio->setChecked(false);
		
		if(value == 1)
			p_Radio->setChecked(true);
		else if(value == 2)
			i_Radio->setChecked(true);
		else if(value == 3)
			d_Radio->setChecked(true);
		
		blockRadioButtons(false);
	}
}

double PIDView::getPGain()
{
	return p_Spin->value();
}

double PIDView::getIGain()
{
	return i_Spin->value();
}

double PIDView::getDGain()
{
	return d_Spin->value();
}

double PIDView::getLimit()
{
	return limit_Spin->value();
}

kf_player::gainSelectEnum PIDView::getFlag()
{
	bool p = p_Radio->isChecked();
	bool i = i_Radio->isChecked();
	bool d = d_Radio->isChecked();
		
	if(!p && !i && !d)
		return kf_player::nonE;
	else if(p && !i && !d)
		return kf_player::rollE;
	else if(!p && i && !d)
		return kf_player::pitchE;
	else //(!p && !i && d)
		return kf_player::yawE;
	
	return kf_player::nonE;
}

void PIDView::p_SpinChanged()
{
	fieldChanged(PIDView::P_VALUE);
	updateBackgroundColor(p_Spin, p_ColorBoundary);
}

void PIDView::i_SpinChanged()
{
	fieldChanged(PIDView::I_VALUE);
	updateBackgroundColor(i_Spin, i_ColorBoundary);
}

void PIDView::d_SpinChanged()
{
	fieldChanged(PIDView::D_VALUE);
	updateBackgroundColor(d_Spin, d_ColorBoundary);
}

void PIDView::limitSpinChanged()
{
	fieldChanged(PIDView::LIMIT);
	updateBackgroundColor(limit_Spin, p_ColorBoundary);
}

void PIDView::p_RadioChanged()
{
	if(p_Radio->isChecked() == true)
	{
		blockRadioButtons(true);
		
		i_Radio->setChecked(false);
		d_Radio->setChecked(false);
		
		blockRadioButtons(false);
	}
	
	fieldChanged(PIDView::PID_FLAG);
}

void PIDView::i_RadioChanged()
{
	if(i_Radio->isChecked() == true)
	{
		blockRadioButtons(true);
		
		p_Radio->setChecked(false);
		d_Radio->setChecked(false);
		
		blockRadioButtons(false);
	}
	
	fieldChanged(PIDView::PID_FLAG);
}

void PIDView::d_RadioChanged()
{
	if(d_Radio->isChecked() == true)
	{
		blockRadioButtons(true);
		
		p_Radio->setChecked(false);
		i_Radio->setChecked(false);
		
		blockRadioButtons(false);
	}
	
	fieldChanged(PIDView::PID_FLAG);
}

void PIDView::updateBackgroundColor(QDoubleSpinBox* spin, float boundary)
{
	double value = fabs(spin->value());
	
	if(value < 0.0001)
		spin->setStyleSheet(zero_sheet);
	else if (value < boundary)
		spin->setStyleSheet(medium_sheet);
	else
		spin->setStyleSheet(large_sheet);
}

void PIDView::blockRadioButtons(bool block)
{
	p_Radio->blockSignals(block);
	i_Radio->blockSignals(block);
	d_Radio->blockSignals(block);
}

PIDView::~PIDView()
{
	delete p_History;
	delete i_History;
	delete d_History;
}