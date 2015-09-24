// Support for KDevelop sessions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef KDEVELOP_H
#define KDEVELOP_H

#include "editor.h"

class QMenu;
class QAction;
class QDir;

namespace rqt_log_viewer
{
namespace editors
{

class KDevelop : public Editor
{
Q_OBJECT
public:
	KDevelop();
	virtual ~KDevelop();

	virtual QAction* createAction(const QString& file, int line);
private Q_SLOTS:
	void openFileInKDevelop();
private:
	void createAction(QMenu* menu, QAction* action, const QDir& sessionDir, const QString& file, int line);
};

}
}

#endif
