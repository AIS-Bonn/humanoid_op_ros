// Single value plot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOT_H
#define PLOT_H

#include <QtCore/QObject>
#include <QtCore/QLinkedList>
#include <QtGui/QColor>
#include <ros/time.h>

#include <boost/circular_buffer.hpp>

class QPainter;
class QRectF;
class QSettings;

namespace plotter
{

class Plot : public QObject
{
Q_OBJECT
public:
	class DataPoint
	{
	public:
		DataPoint()
		 : time(0), value(0), labelY(0)
		{}

		DataPoint(const ros::Time _time, double _value, unsigned int _labelY = 0)
		 : time(_time), value(_value), labelY(_labelY)
		{}

		// Timestamp of the data point
		ros::Time time;

		// Plot value (NAN if this is an event)
		double value;

		// Label offset in lines, up is positive (only used if this is an event)
		unsigned int labelY;

		bool operator<(const DataPoint& other) const
		{ return time < other.time; }
	};

	typedef boost::circular_buffer<DataPoint> DataBuffer;

	// Iterator providing access to parent Plot object
	class LinkedBufferIterator : public DataBuffer::const_iterator
	{
	public:
		LinkedBufferIterator()
		 : m_plot(0)
		{}

		explicit LinkedBufferIterator(const DataBuffer::const_iterator& it, const Plot* plot)
		 : DataBuffer::const_iterator(it)
		 , m_plot(plot)
		{}

		inline const Plot* plot() const
		{ return m_plot; }

		bool isValid() const
		{ return m_plot && (*this) != m_plot->m_buf.end(); }
	private:
		const Plot* m_plot;
	};

	Plot(const QString& name, Plot* parent = 0);
	virtual ~Plot();

	virtual void draw(QPainter* painter, const ros::Time& base, const QRectF& rect, bool dots) const;
	virtual QVariant displayData(int role) const;

	//! @name Plot properties
	//@{
	void setUsedSettings(QSettings* settings);
	inline QSettings* usedSettings()
	{ return m_settings; }

	void serialize();

	void setEnabled(bool enabled);
	inline bool isEnabled() const
	{ return m_enabled; }

	void setName(const QString& name);
	inline QString name() const
	{ return m_name; }

	void setColor(const QColor& color);
	inline QColor color() const
	{ return m_color; }

	void setPaused(bool paused);
	inline bool paused() const
	{ return m_paused; }

	void setIsEventChannel(bool eventChannel);
	inline bool isEventChannel() const
	{ return m_isEventChannel; }
	//@}

	//! @name Plot data interface
	//@{
	double lastValue() const
	{ return (*m_buf.rbegin()).value; }

	ros::Time lastTime() const;

	inline bool hasData() const
	{ return m_hasData; }

	void put(const ros::Time& time, double value, unsigned int labelY = 0, bool notify = true);

	double value(const ros::Time& time) const;
	//@}

	//! @name Plot tree traversal
	//@{
	void setParent(Plot* parent);

	QList<Plot*> childPlots()
	{ return m_children; }

	int childCount() const
	{ return m_children.count(); }

	const Plot* child(int idx) const
	{ return m_children[idx]; }

	int indexOfChild(const Plot* plot) const
	{ return m_children.indexOf((Plot*)plot); }

	/**
	 * Get plot path
	 **/
	QString path() const;

	/**
	 * Find plot by relative path.
	 *
	 * @param path Path (e.g. "Joint States/Positions/right_knee_pitch")
	 **/
	Plot* findPlotByPath(const QString& path) const;

	Plot* findOrCreatePlotByPath(const QString& path);

	ros::Time recursiveLastTime() const;

	enum FindFlag
	{
		FindNoFlag      = 0,
		FindRecursive   = (1 << 0),
		FindOnlyVisible = (1 << 1),
	};
	Q_DECLARE_FLAGS(FindFlags, FindFlag)

	LinkedBufferIterator findDataPointAt(const ros::Time& time, double value, FindFlags flags = FindNoFlag);
	QLinkedList<LinkedBufferIterator> iterators(FindFlags flags = FindRecursive) const;
	//@}
Q_SIGNALS:
	void changed(Plot* source);
	void beginAddChild(Plot* parent, int atIndex);
	void childAdded(Plot* parent);
	void hierarchyChanged();
protected:
private:
	DataBuffer m_buf;
	QString m_name;
	QSettings* m_settings;

	bool m_enabled;
	QList<Plot*> m_children;
	bool m_hasData;
	QColor m_color;
	bool m_paused;
	bool m_isEventChannel;

	void addChild(Plot* plot);
};

}

Q_DECLARE_OPERATORS_FOR_FLAGS(plotter::Plot::FindFlags)

#endif
