#ifndef HEADERVIEW_H
#define HEADERVIEW_H

// Widget to edit motionName, preState, playState, postState
// If some variable is being set to invalid value, corresponding QLineEdit becomes red
// If value is valid, QLineEdit is white
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <QObject>

namespace Ui
{
	class HeaderView; 
}

class HeaderView : public QWidget
{
Q_OBJECT
public:
	HeaderView(QWidget *parent = 0);
	
	void enableEdit(bool flag); // set all edits editable/not editable
	void setValues(std::string const name, std::string const preState
		, std::string const playState, std::string const postState);
	
	// If values are valid - return true. If not - show warning and ask if user still would like to save it
	// If user click 'yes' - return true. Otherwise - false
	bool requestSave();
	
	void setFileName(QString name);
	std::string setFileNameFromPath(QString name); // Parse name from full path to file
	
	std::string getName();
	std::string getPreState();
	std::string getPlayState();
	std::string getPostState();
	double getFactor();
	
	QString getFileName();
	QString getWarningString(); // String includes warning for each invalid value. Empty if everything is ok
	
	virtual ~HeaderView();
	
Q_SIGNALS:
	void requestUpdate(); // Emmited when data is changed
	
public Q_SLOTS:
	void checkValues();
	
private:
	Ui::HeaderView *ui;
	
	QString fileName;
	QString warningString;
};


#endif // HEADERVIEW_H
