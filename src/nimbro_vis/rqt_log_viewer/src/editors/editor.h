// File editor base class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef EDITOR_H
#define EDITOR_H

#include <QtCore/QObject>

class QAction;

namespace rqt_log_viewer
{
namespace editors
{

class Editor : public QObject
{
public:
	Editor() {}
	virtual ~Editor() {}

	virtual QAction* createAction(const QString& file, int line) = 0;
};

}
}

#endif
