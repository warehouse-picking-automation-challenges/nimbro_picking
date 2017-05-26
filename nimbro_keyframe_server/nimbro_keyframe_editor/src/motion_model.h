// Qt model for a Motion
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CENTAURO_KEYFRAME_SERVER_MOTION_MODEL_H
#define CENTAURO_KEYFRAME_SERVER_MOTION_MODEL_H

#include <QtCore/QAbstractTableModel>

#include <nimbro_keyframe_server/MotionMsg.h>
#include <nimbro_keyframe_server/motion.h>

#include <nimbro_keyframe_server/ExecutionFeedback.h>

namespace nimbro_keyframe_editor
{

class MotionModel : public QAbstractTableModel
{
Q_OBJECT
public:
	explicit MotionModel(QObject* parent = 0);
	virtual ~MotionModel();

	void load(const nimbro_keyframe_server::MotionMsg& msg);
	void load(const nimbro_keyframe_server::Motion& motion);
	nimbro_keyframe_server::MotionMsg toMsg() const;

	void clear();

	void duplicateKeyFrame(int index);
	void removeKeyFrame(int index);

	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const;
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const;

	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;

	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

	virtual Qt::ItemFlags flags(const QModelIndex& index) const;
	virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);

	inline nimbro_keyframe_server::Motion& motion()
	{ return m_motion; }

	nimbro_keyframe_server::KeyFrame* keyFrameForIndex(const QModelIndex& index);

	void setFeedback(const nimbro_keyframe_server::ExecutionFeedback& feedback);
	void resetFeedback();
private:
	enum Column
	{
		COL_LABEL,
		COL_LIN_SPEED,
		COL_ANG_SPEED,
		COL_LIN_ACC,
		COL_ANG_ACC,
		COL_JOINT_SPEED,
		COL_JOINT_ACC,
		COL_TORQ_PROP,
		COL_COUNT
	};

	nimbro_keyframe_server::Motion m_motion;
	nimbro_keyframe_server::ExecutionFeedback m_feedback;
};

}

#endif
