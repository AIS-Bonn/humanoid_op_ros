#include <trajectory_editor_2/headerview.h>

#include "ui_headerview.h"

#include <QLineEdit>
#include <QObject>
#include <QMessageBox>

HeaderView::HeaderView(QWidget *parent) : QWidget(parent)
{
	ui = new Ui::HeaderView;
	ui->setupUi(this);
	
	connect(ui->nameLineEdit, SIGNAL(textEdited(QString)), this, SLOT(checkValues()));
	connect(ui->preStateLineEdit, SIGNAL(textEdited(QString)), this, SLOT(checkValues()));
	connect(ui->playStateLineEdit, SIGNAL(textEdited(QString)), this, SLOT(checkValues()));
	connect(ui->postStateLineEdit, SIGNAL(textEdited(QString)), this, SLOT(checkValues()));
}

void HeaderView::setValues(std::string const name, std::string const preState
	, std::string const playState, std::string const postState)
{
	ui->nameLineEdit->setText(QString::fromStdString(name));
	ui->preStateLineEdit->setText(QString::fromStdString(preState));
	ui->playStateLineEdit->setText(QString::fromStdString(playState));
	ui->postStateLineEdit->setText(QString::fromStdString(postState));
	
	this->checkValues();
}

void  HeaderView::setFileName(QString name)
{
	this->fileName = name;
	this->checkValues();
}

std::string HeaderView::setFileNameFromPath(QString name)
{	
	QStringList pieces = name.split( "/" );
	QStringList pieces2 = pieces.last().split( "." );
	
	this->fileName = pieces2.first();
	//this->checkValues();
	
	return fileName.toStdString();
}

void HeaderView::checkValues()
{
	this->requestUpdate();
	
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
	this->checkValues();
	
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
	return this->warningString;
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

std::string HeaderView::getName()
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
	return this->fileName;
}

HeaderView::~HeaderView()
{
	delete ui;
}
