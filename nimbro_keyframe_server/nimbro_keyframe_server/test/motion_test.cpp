// Test to load and save motion files
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <nimbro_keyframe_server/motion.h>

#include <catch_ros/catch.hpp>

using namespace nimbro_keyframe_server;

void initJointGroups(KeyFrame* k)
{
	Eigen::Affine3d state(Eigen::Affine3d::Identity());
	state.translate(Eigen::Vector3d(1.2, 2.3, -2.1));
	state.rotate(Eigen::Matrix3d(
		Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(-1.004, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(2.05, Eigen::Vector3d::UnitX())
	));


	KeyFrame::JointGroup g(KeyFrame::IS_JOINT_SPACE);
	k->addJointGroup("one", g);
	g.setInterpolationSpace(KeyFrame::IS_CARTESIAN);
	g.setState(state);
	k->addJointGroup("two", g);
	g.setInterpolationSpace(KeyFrame::IS_NONE);
	g.setState(Eigen::Affine3d::Identity());
	k->addJointGroup("three", g);

	k->setJointPosition("joint_01", 0.01);
	k->setJointPosition("joint_02", 1.24);
	k->setJointPosition("joint_03", 2.37);
	k->setJointPosition("joint_04", -1.91);
	k->setJointPosition("joint_05", -0.01);
}

void initKeyFrame(Motion* m)
{
	KeyFrame k;
	k.setLabel("keyframe_1");
	k.setAngularVelocity(1.0);
	k.setCartLinearVelocity(0.5);
	k.setTorqueProportion(-1.0);
	initJointGroups(&k);
	m->push_back(k);
}

Motion initMotion()
{
	Motion m("test");
	initKeyFrame(&m);

	return m;
}

void addReferenceObjectToMotion(Motion* m)
{
	m->referencePose = Eigen::Affine3d::Identity();
	m->referencePose.translate(Eigen::Vector3d(0.9, -1.2, 2.3));
	m->referencePose.rotate(Eigen::Matrix3d(
		Eigen::AngleAxisd(2.1, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(1.001, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(-2.1, Eigen::Vector3d::UnitX())
	));

	m->referenceObjectMesh = "package://this/is/a/test/";
}

void testPose(const Eigen::Affine3d& base, const Eigen::Affine3d& target)
{
	Eigen::Matrix4d basePose = base.matrix();
	Eigen::Matrix4d targetPose = target.matrix();
	int i, j;
	for(i = 0; i < 4; ++i)
	{
		for(j = 0; j < 4; ++j)
		{
			CAPTURE(i);
			CAPTURE(j);
			CHECK(basePose(i,j) == Approx(targetPose(i,j)));
		}
	}

}

void testJointPositions(const KeyFrame& base, const KeyFrame& target)
{
	const KeyFrame::JointMap baseJoints = base.jointPositions();
	const KeyFrame::JointMap targetJoints = target.jointPositions();
	CHECK(baseJoints.size() == targetJoints.size());
	for(const auto& pair : baseJoints)
	{
		const auto it = targetJoints.find(pair.first);
		CHECK(it != targetJoints.end());
		CHECK(pair.second == Approx(it->second));
	}
}

void testJointGroups(const KeyFrame& base, const KeyFrame& target)
{
	const KeyFrame::GroupMap baseGroups = base.jointGroups();
	const KeyFrame::GroupMap targetGroups = target.jointGroups();
	for(const auto& pair : baseGroups)
	{
		//NOTE(sebastian): We allow not interpolated Keyframes to be dropped
		if(pair.second.interpolationSpace() == KeyFrame::IS_NONE)
			continue;

		const auto it = targetGroups.find(pair.first);
		CHECK(it != targetGroups.end());
		const auto& baseGroup = pair.second;
		const auto& targetGroup = it->second;
		CHECK(baseGroup.interpolationSpace() == targetGroup.interpolationSpace());
		testPose(baseGroup.state(), targetGroup.state());
	}
}

void testMotions(const Motion& base, const Motion& target)
{
	INFO("Checking Motion Metadata.");
	REQUIRE(base.isValid());
	REQUIRE(target.isValid());
	CHECK(base.name() == target.name());
	CHECK(base.size() == target.size());
	CHECK(base.referenceObjectMesh == target.referenceObjectMesh);
	if(std::isfinite(base.referencePose.translation().x()))
	{
		INFO("Found Valid Reference Pose. Checking.");
		CHECK(std::isfinite(base.referencePose.translation().y()));
		CHECK(std::isfinite(base.referencePose.translation().z()));
		CHECK(std::isfinite(target.referencePose.translation().x()));
		CHECK(std::isfinite(target.referencePose.translation().y()));
		CHECK(std::isfinite(target.referencePose.translation().z()));
		CHECK(base.referenceObjectMesh == target.referenceObjectMesh);
		CHECK(!base.referenceObjectMesh.empty());
		CHECK(!target.referenceObjectMesh.empty());
		testPose(base.referencePose, target.referencePose);
	}
	else
	{
		INFO("No Valid Reference Pose. Checking consistency.");
		CHECK(!std::isfinite(base.referencePose.translation().y()));
		CHECK(!std::isfinite(base.referencePose.translation().z()));
		CHECK(!std::isfinite(target.referencePose.translation().x()));
		CHECK(!std::isfinite(target.referencePose.translation().y()));
		CHECK(!std::isfinite(target.referencePose.translation().z()));
	}

	INFO("Checking KeyFrames.");
	size_t i;
	for(i = 0; i < base.size(); ++i)
	{
		const KeyFrame& baseFrame = base[i];
		const KeyFrame& targetFrame = target[i];
		CHECK(baseFrame.label() == targetFrame.label());
		CHECK(baseFrame.angularVelocity() == Approx(targetFrame.angularVelocity()));
		CHECK(baseFrame.linearVelocity() == Approx(targetFrame.linearVelocity()));
		CHECK(baseFrame.torqueProportion() == Approx(targetFrame.torqueProportion()));
		// Test in both directions to be sure!
		testJointPositions(baseFrame, targetFrame);
		testJointPositions(targetFrame, baseFrame);
		testJointGroups(baseFrame, targetFrame);
		testJointGroups(targetFrame, baseFrame);
	}
}



TEST_CASE("base_motion from and to yaml", "[Motion]")
{
	Motion m = initMotion();
	REQUIRE(m.isValid());
	std::stringstream serialized(m.serializeToYAML());
	CAPTURE(serialized.str());
	Motion target = Motion::loadFromYAML(serialized);
	testMotions(m, target);
}

TEST_CASE("base_motion from and to msg", "[Motion]")
{
	Motion base = initMotion();
	REQUIRE(base.isValid());
	MotionMsg msg = base.toMsg();
	Motion target = Motion::loadFromMsg(msg);
	testMotions(base, target);
}

TEST_CASE("reference_object from and to yaml", "[Motion]")
{
	Motion base = initMotion();
	addReferenceObjectToMotion(&base);
	REQUIRE(base.isValid());
	REQUIRE(std::isfinite(base.referencePose.translation().x()));
	std::stringstream serialized(base.serializeToYAML());
	CAPTURE(serialized.str());
	Motion target = Motion::loadFromYAML(serialized);
	testMotions(base, target);
}

TEST_CASE("reference_object from and to msg", "[Motion]")
{
	Motion base = initMotion();
	addReferenceObjectToMotion(&base);
	REQUIRE(base.isValid());
	REQUIRE(std::isfinite(base.referencePose.translation().x()));
	MotionMsg msg = base.toMsg();
	Motion target = Motion::loadFromMsg(msg);
	testMotions(base, target);
}
