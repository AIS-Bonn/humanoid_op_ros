#include <trajectory_editor/headerview.h>

#include "ui_headerview.h"

#include <QLineEdit>
#include <QObject>
#include <QMessageBox>

#include <stdio.h>

HeaderView::HeaderView(QWidget *parent) : QWidget(parent)
{
	ui = new Ui::HeaderView;
	ui->setupUi(this);
	
	connect(ui->nameLineEdit, SIGNAL(textChanged(QString)), this, SLOT(handleFieldChanged()));
	connect(ui->preStateLineEdit, SIGNAL(textChanged(QString)), this, SLOT(handleFieldChanged()));
	connect(ui->playStateLineEdit, SIGNAL(textChanged(QString)), this, SLOT(handleFieldChanged()));
	connect(ui->postStateLineEdit, SIGNAL(textChanged(QString)), this, SLOT(handleFieldChanged()));
	
	connect(ui->enablePIDCheckbox, SIGNAL(toggled(bool)), this, SLOT(handleFieldChanged()));
	
	this->enableEdit(false);
}

void HeaderView::setData(HeaderData data)
{
	this->setValues(data.motion_name, data.pre_state, data.play_state, data.post_state, data.pid_enabled);
}

void HeaderView::setValues(std::string const name, std::string const preState
	, std::string const playState, std::string const postState, bool pidEnabled)
{
	// TODO refactor this
	this->enableEdit(true);
	
	ui->enablePIDCheckbox->blockSignals(true);
	ui->nameLineEdit->blockSignals(true);
	ui->preStateLineEdit->blockSignals(true);
	ui->playStateLineEdit->blockSignals(true);
	ui->postStateLineEdit->blockSignals(true);
	
	ui->nameLineEdit->setText(QString::fromStdString(name));
	ui->preStateLineEdit->setText(QString::fromStdString(preState));
	ui->playStateLineEdit->setText(QString::fromStdString(playState));
	ui->postStateLineEdit->setText(QString::fromStdString(postState));
	ui->enablePIDCheckbox->setChecked(pidEnabled);
	
	ui->enablePIDCheckbox->blockSignals(false);
	ui->nameLineEdit->blockSignals(false);
	ui->preStateLineEdit->blockSignals(false);
	ui->playStateLineEdit->blockSignals(false);
	ui->postStateLineEdit->blockSignals(false);
	
	pidEnabledChanged(ui->enablePIDCheckbox->isChecked());
	checkValues();
}

void HeaderView::setFileName(QString name)
{
	fileName = name;
	checkValues();
}

std::string HeaderView::getNameForMirrored()
{
	QString name = ui->nameLineEdit->text();
	std::size_t pos = 0;
	
	// Check for left
	pos = name.toStdString().find("left");
	if(pos != std::string::npos)
	{
		name.remove(pos, 4);
		name.insert(pos, QString("right"));
		
		return name.toStdString();
	}
	
	// Check for right
	pos = name.toStdString().find("right");
	if(pos != std::string::npos)
	{
		name.remove(pos, 5);
		name.insert(pos, QString("left"));
		
		return name.toStdString();
	}
	
	name.append("_mirrored");
	return name.toStdString();
}


std::string HeaderView::getFileNameFromPath(QString path)
{	
	QStringList pieces = path.split("/");
	QStringList pieces2 = pieces.last().split(".");
	
	return pieces2.first().toStdString();
}

void HeaderView::handleFieldChanged()
{
	HeaderData data;
	
	data.motion_name = this->getMotionName();
	data.pre_state   = this->getPreState();
	data.play_state  = this->getPlayState();
	data.post_state  = this->getPostState();
	data.pid_enabled = this->getPIDEnabled();
	
	checkValues();
	dataChanged(data);
	pidEnabledChanged(ui->enablePIDCheckbox->isChecked());
}

void HeaderView::checkValues()
{
	QString errorSheet("QLineEdit { background: rgb(235, 150, 150); }");
	QString okSheet("QLineEdit { background: rgb(255, 255, 255); }");
	
	warningString.clear();
	
	/*if(ui->nameLineEdit->text() != fileName) // check if motion name is valid
	{
		ui->nameLineEdit->setStyleSheet(errorSheet);
		warningString.append("Motion name is not valid\n");
	}
	else
		ui->nameLineEdit->setStyleSheet(okSheet);*/
	
	if(ui->preStateLineEdit->text() == "") // check if preState is valid
	{
		ui->preStateLineEdit->setStyleSheet(errorSheet);
		warningString.append("PreState is not valid\n");
	}
	else
		ui->preStateLineEdit->setStyleSheet(okSheet);
	
	if(ui->playStateLineEdit->text() == "") // check if playState is valid
	{
		ui->playStateLineEdit->setStyleSheet(errorSheet);
		warningString.append("PlayState is not valid\n");
	}
	else
		ui->playStateLineEdit->setStyleSheet(okSheet);
	
	if(ui->postStateLineEdit->text() == "") // check if postState is valid
	{
		ui->postStateLineEdit->setStyleSheet(errorSheet);
		warningString.append("PostState is not valid\n");
	}
	else
		ui->postStateLineEdit->setStyleSheet(okSheet);
}

bool HeaderView::requestSave()
{
	checkValues();
	
	if(!warningString.isEmpty()) // if there are warnings - show warning massage
	{
		warningString.prepend("Errors were found:\n\n");
		warningString.append("\nSave it anyway?");
		
		QMessageBox messageBox;
		messageBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard);
		messageBox.setDefaultButton(QMessageBox::Discard);
		messageBox.setWindowTitle("Warning!");
		messageBox.setText(warningString);
		
		int buttonPressed = messageBox.exec();
		if(buttonPressed == QMessageBox::Discard) // "Dont save" pressed
			return false;
	}
	
	return true;
}

QString HeaderView::getWarningString()
{
	return warningString;
}

void HeaderView::enableEdit(bool flag)
{
	ui->nameLineEdit->setEnabled(flag);
	ui->preStateLineEdit->setEnabled(flag);
	ui->playStateLineEdit->setEnabled(flag);
	ui->postStateLineEdit->setEnabled(flag);
}

double HeaderView::getFactor()
{
	return ui->factorSpinBox->value();
}

bool HeaderView::getPIDEnabled()
{
	return ui->enablePIDCheckbox->isChecked();
}

std::string HeaderView::getMotionName()
{
	return ui->nameLineEdit->text().toStdString();
}

std::string HeaderView::getPreState()
{
	return ui->preStateLineEdit->text().toStdString();
}

std::string HeaderView::getPlayState()
{
	return ui->playStateLineEdit->text().toStdString();
}

std::string HeaderView::getPostState()
{
	return ui->postStateLineEdit->text().toStdString();
}

QString HeaderView::getFileName()
{
	return fileName;
}

HeaderView::~HeaderView()
{
	delete ui;
}
