// Qt model for a Motion
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "motion_model.h"

#include <QColor>
#include <QBrush>

namespace nimbro_keyframe_editor
{

MotionModel::MotionModel(QObject* parent)
 : QAbstractTableModel(parent)
{
}

MotionModel::~MotionModel()
{
}

int MotionModel::rowCount(const QModelIndex& parent) const
{
	return m_motion.size();
}

int MotionModel::columnCount(const QModelIndex& parent) const
{
	return COL_COUNT;
}

QVariant MotionModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(orientation == Qt::Horizontal && role == Qt::DisplayRole)
	{
		switch(section)
		{
			case COL_LABEL: return "Label";
			case COL_LIN_SPEED: return "L Vel";
			case COL_ANG_SPEED: return "A Vel";
			case COL_LIN_ACC: return "L Acc";
			case COL_ANG_ACC: return "A Acc";
			case COL_JOINT_SPEED: return "JS Vel";
			case COL_JOINT_ACC: return "JS Acc";
			case COL_TORQ_PROP: return "Torque";
		}
	}

	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant MotionModel::data(const QModelIndex& index, int role) const
{
	const nimbro_keyframe_server::KeyFrame& kf = m_motion[index.row()];

	switch(role)
	{
		case Qt::DisplayRole:
			switch(index.column())
			{
				case COL_LABEL:
					return QString::fromStdString(kf.label());
				case COL_LIN_SPEED:
					return QString::number(kf.linearVelocity(), 'f', 3);
				case COL_ANG_SPEED:
					return QString::number(kf.angularVelocity(), 'f', 3);
				case COL_LIN_ACC:
					return QString::number(kf.linearAcceleration(), 'f', 3);
				case COL_ANG_ACC:
					return QString::number(kf.angularAcceleration(), 'f' ,3);
				case COL_JOINT_SPEED:
					return QString::number(kf.jointSpaceVelocity(), 'f', 3);
				case COL_JOINT_ACC:
					return QString::number(kf.jointSpaceAcceleration(), 'f', 3);
				case COL_TORQ_PROP:
					return QString::number(kf.torqueProportion(), 'f', 2);
			}
			break;
		case Qt::EditRole:
			switch(index.column())
			{
				case COL_LABEL:
					return QString::fromStdString(kf.label());
				case COL_LIN_SPEED:
					return QString::number(kf.linearVelocity(), 'f', 3);
				case COL_ANG_SPEED:
					return QString::number(kf.angularVelocity(), 'f', 3);
				case COL_LIN_ACC:
					return QString::number(kf.linearAcceleration(), 'f', 3);
				case COL_ANG_ACC:
					return QString::number(kf.angularAcceleration(), 'f' ,3);
				case COL_JOINT_SPEED:
					return QString::number(kf.jointSpaceVelocity(), 'f', 3);
				case COL_JOINT_ACC:
					return QString::number(kf.jointSpaceAcceleration(), 'f', 3);
				case COL_TORQ_PROP:
					return QString::number(kf.torqueProportion(), 'f', 2);
			}
			break;
		case Qt::BackgroundRole:
			if(m_feedback.playing && (int)m_feedback.keyframe_index == index.row())
			{
				if(
					   (index.column() == COL_LIN_SPEED   && (m_feedback.active_limits & m_feedback.LIN_VEL_LIMIT))
					|| (index.column() == COL_LIN_ACC     && (m_feedback.active_limits & m_feedback.LIN_ACC_LIMIT))
					|| (index.column() == COL_ANG_SPEED   && (m_feedback.active_limits & m_feedback.ANG_VEL_LIMIT))
					|| (index.column() == COL_ANG_ACC     && (m_feedback.active_limits & m_feedback.ANG_ACC_LIMIT))
					|| (index.column() == COL_JOINT_SPEED && (m_feedback.active_limits & m_feedback.JS_VEL_LIMIT))
					|| (index.column() == COL_JOINT_ACC   && (m_feedback.active_limits & m_feedback.JS_ACC_LIMIT))
				)
					return QBrush(QColor(128,128,255));
				else
					return QBrush(QColor(255,255,128));
			}
			break;
	}

	return QVariant();
}

void MotionModel::load(const nimbro_keyframe_server::MotionMsg& msg)
{
	load(nimbro_keyframe_server::Motion::loadFromMsg(msg));
}

void MotionModel::load(const nimbro_keyframe_server::Motion& motion)
{
	beginResetModel();
	m_motion = motion;
	endResetModel();
}

void MotionModel::clear()
{
	beginResetModel();
	m_motion = nimbro_keyframe_server::Motion();
	endResetModel();
}

nimbro_keyframe_server::KeyFrame* MotionModel::keyFrameForIndex(const QModelIndex& index)
{
	return &m_motion[index.row()];
}

void MotionModel::duplicateKeyFrame(int index)
{
	beginInsertRows(QModelIndex(), index, index);
	nimbro_keyframe_server::KeyFrame tpl = m_motion[index];
	tpl.setLabel("copy of " + tpl.label());
	m_motion.insert(m_motion.begin() + index + 1, tpl);
	endInsertRows();
}

void MotionModel::removeKeyFrame(int index)
{
	beginRemoveRows(QModelIndex(), index, index);
	m_motion.erase(m_motion.begin() + index);
	endRemoveRows();
}

Qt::ItemFlags MotionModel::flags(const QModelIndex& index) const
{
	Qt::ItemFlags flags = QAbstractItemModel::flags(index);

	flags |= Qt::ItemIsEditable;

	return flags;
}

bool MotionModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if(role != Qt::EditRole)
		return false;

	nimbro_keyframe_server::KeyFrame* frame = &m_motion[index.row()];

	switch(index.column())
	{
		case COL_LABEL:
			frame->setLabel(value.toString().toStdString());
			dataChanged(index, index);
			return true;
			break;
		case COL_LIN_SPEED:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setCartLinearVelocity(val);
			dataChanged(index, index);
			return true;
			break;
		}
		case COL_ANG_SPEED:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setAngularVelocity(val);
			dataChanged(index, index);
			return true;
			break;
		}
		case COL_LIN_ACC:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setLinearAcceleration(val);
			dataChanged(index, index);
			return true;
			break;

		}
		case COL_ANG_ACC:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setAngularAcceleration(val);
			dataChanged(index, index);
			return true;
			break;

		}
		case COL_JOINT_SPEED:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setJointSpaceVelocity(val);
			dataChanged(index, index);
			return true;
			break;

		}
		case COL_JOINT_ACC:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;

			frame->setJointSpaceAcceleration(val);
			dataChanged(index, index);
			return true;
			break;

		}

		case COL_TORQ_PROP:
		{
			bool ok = true;
			double val = value.toDouble(&ok);
			if(!ok)
				return false;
			val = val > 1.0 ? 1.0 : val;
			frame->setTorqueProportion(val);
			dataChanged(index, index);
			break;
		}
	}

	return false;
}

void MotionModel::setFeedback(const nimbro_keyframe_server::ExecutionFeedback& feedback)
{
	int oldRow = (m_feedback.playing ? m_feedback.keyframe_index : -1);

	m_feedback = feedback;

	if(oldRow != -1)
		dataChanged(index(oldRow, 0), index(oldRow, COL_COUNT-1));

	if(m_feedback.playing)
		dataChanged(index(m_feedback.keyframe_index, 0), index(m_feedback.keyframe_index, COL_COUNT-1));
}

void MotionModel::resetFeedback()
{
	int oldRow = (m_feedback.playing ? m_feedback.keyframe_index : -1);

	m_feedback = nimbro_keyframe_server::ExecutionFeedback();

	if(oldRow != -1)
		dataChanged(index(oldRow, 0), index(oldRow, COL_COUNT-1));
}

}
