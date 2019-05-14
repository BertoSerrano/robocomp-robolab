//
// Created by robolab on 4/09/18.
//

#ifndef PROJECT_DOUBLEBUFFERCONVERTERS_H
#define PROJECT_DOUBLEBUFFERCONVERTERS_H

#include <qdebug.h>


//TODO: This implementantions should go it's own file
class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
{
private:
	std::size_t size=0;
public:
	ByteSeqConverter(std::size_t _size)
	{
		size=_size;
	}
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData, bool clear=false)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData, bool clear=false)
    {
        return false;
    }
};

class PointSeqConverter : public Converter<astra::PointFrame, RoboCompRGBD::PointSeq>
{
private:
	std::size_t size=0;
public:
	PointSeqConverter(std::size_t _size)
	{
		size=_size;
	}
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::PointSeq &oTypeData, bool clear=false)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
//            std::cout<<"iTypeData length "<<iTypeData.length()<<endl;
//            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.length());
            int non0 = 0;
			for(uint i = 0; i< iTypeData.length(); i++)
			{
				oTypeData[i].x = iTypeData.data()[i].x;
				oTypeData[i].y = iTypeData.data()[i].y;
				oTypeData[i].z = iTypeData.data()[i].z;
				if(iTypeData.data()[i].x != 0 or iTypeData.data()[i].y != 0 or iTypeData.data()[i].z != 0)
				{
					non0++;
				}
				std::cout<<non0<<" non 0 values in Point Stream";
			}
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::PointSeq &oTypeData, astra::PointFrame &iTypeData, bool clear=false)
    {
        return false;
    }
};


class PointStreamConverter : public Converter<astra::PointFrame, RoboCompRGBD::imgType>
{
private:
	std::size_t size=0;
public:
	PointStreamConverter(std::size_t _size)
	{
		size=_size;
	}
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::imgType &oTypeData, bool clear=false)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
//            std::cout<<"iTypeData length "<<iTypeData.length()<<endl;
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.length()*3*4);
//            auto point = iTypeData.data()[10];
//			std::cout<<"Point ("<<point.x<<", "<<point.y<<", "<<point.z<<")"<<std::endl;
//            int non0 = 0;
//			for(uint i = 0; i< iTypeData.length(); i++)
//			{
//				oTypeData[i].x = iTypeData.data()[i].x;
//				oTypeData[i].y = iTypeData.data()[i].y;
//				oTypeData[i].z = iTypeData.data()[i].z;
////				if(iTypeData.data()[i].x != 0 or iTypeData.data()[i].y != 0 or iTypeData.data()[i].z != 0)
////				{
////					non0++;
////				}
////				std::cout<<non0<<" non 0 values in Point Stream"<<endl;
//			}
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::PointFrame &iTypeData, bool clear=false)
    {
        return false;
    }
};

class ColorSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::ColorSeq>
{
private:
	std::size_t size=0;
public:
	ColorSeqConverter(std::size_t _size)
	{
		size=_size;
	}
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::ColorSeq &oTypeData, bool clear=false)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::ColorSeq &oTypeData, astra::ColorFrame &iTypeData, bool clear=false)
    {
        return false;
    }
};
//class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
//{
//private:
//	std::size_t size=0;
//public:
//	ByteSeqConverter(std::size_t _size)
//	{
//		size=_size;
//	}
//    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData, bool clear=false)
//    {
//		  if(oTypeData.size()<this->size)
// {
// oTypeData.resize(size);
//        if (iTypeData.is_valid())
//        {
//
//            //            this->resize(d.width() * d.height()*data_size);
//            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
//            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
//            return true;
//        }
//        return false;
//    }
//
//    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData, bool clear=false)
//    {
//        return false;
//    }
//};

class FloatSeqConverter : public Converter<astra::DepthFrame, RoboCompRGBD::DepthSeq>
{
private:
	std::size_t size=0;
public:
	FloatSeqConverter(std::size_t _size)
	{
		size=_size;
	}
    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData, bool clear=false)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {
            std::copy(&iTypeData.data()[0], &iTypeData.data()[0]+(iTypeData.width()*iTypeData.height()), std::begin(oTypeData));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::DepthSeq &oTypeData, astra::DepthFrame &iTypeData, bool clear=false)
    {
        return false;
    }
};



class BodiesPeopleConverter : public Converter<astra::BodyFrame, RoboCompHumanTracker::PersonList>
{
	std::map<astra::JointType, ::std::string> joint2String = {
			std::make_pair(astra::JointType::Head,"Head"),
			std::make_pair(astra::JointType::Neck,"Neck"),
			std::make_pair(astra::JointType::ShoulderSpine,"ShoulderSpine"),
			std::make_pair(astra::JointType::LeftShoulder,"LeftShoulder"),
			std::make_pair(astra::JointType::LeftElbow,"LeftElbow"),
			std::make_pair(astra::JointType::LeftWrist,"LeftWrist"),
			std::make_pair(astra::JointType::LeftHand,"LeftHand"),
			std::make_pair(astra::JointType::RightShoulder,"RightShoulder"),
			std::make_pair(astra::JointType::RightElbow,"RightElbow"),
			std::make_pair(astra::JointType::RightWrist,"RightWrist"),
			std::make_pair(astra::JointType::RightHand,"RightHand"),
			std::make_pair(astra::JointType::MidSpine,"MidSpine"),
			std::make_pair(astra::JointType::BaseSpine,"BaseSpine"),
			std::make_pair(astra::JointType::LeftHip,"LeftHip"),
			std::make_pair(astra::JointType::LeftKnee,"LeftKnee"),
			std::make_pair(astra::JointType::LeftFoot,"LeftFoot"),
			std::make_pair(astra::JointType::RightHip,"RightHip"),
			std::make_pair(astra::JointType::RightKnee,"RightKnee"),
			std::make_pair(astra::JointType::RightFoot,"RightFoot")
	};

public:

	bool ItoO(const astra::BodyFrame &iTypeData, RoboCompHumanTracker::PersonList &oTypeData, bool clear=false)
	{
		if(!clear)
		{
			if (iTypeData.is_valid())
			{
				const auto &bodies = iTypeData.bodies();
				if (bodies.empty())
					return false;

				for (auto &body : bodies) {
					//            qDebug()<<"------------------ Found person " << body.id()<<"--------------------";

					RoboCompHumanTracker::TPerson person;
					auto status = body.status();

					switch (status) {
						case astra::BodyStatus::NotTracking:
							person.state = RoboCompHumanTracker::TrackingState::NotTracking;
							break;
						case astra::BodyStatus::TrackingLost:
							person.state = RoboCompHumanTracker::TrackingState::TrackingLost;
							break;
						case astra::BodyStatus::TrackingStarted:
							person.state = RoboCompHumanTracker::TrackingState::TrackingStarted;
							break;
						case astra::BodyStatus::Tracking:
							person.state = RoboCompHumanTracker::TrackingState::Tracking;
							break;
						default:
							qDebug() << "Invalid body state";
					}


					RoboCompHumanTracker::jointListType joints_list;
					RoboCompHumanTracker::jointListType joints_depth;

					const auto &joints = body.joints();

					if (!joints.empty()) {

						for (const auto &j : joints) {
							if (j.status() == astra::JointStatus::Tracked or
								j.status() == astra::JointStatus::LowConfidence) {
								auto &jnt = j.world_position();
								auto &jntdepth = j.depth_position();

								joint pointindepth;
								pointindepth.push_back(jntdepth.x);
								pointindepth.push_back(jntdepth.y);


								joint JointP;
								JointP.push_back(jnt.x);
								JointP.push_back(jnt.y);
								JointP.push_back(jnt.z);

								astra::JointType type = j.type();
								std::string typejoint;

								typejoint = joint2String.at(type);
								joints_list[typejoint] = JointP;
								joints_depth[typejoint] = pointindepth;

							}
						}
					} else
						qDebug() << "Joints is empty";

					person.joints = joints_list;
					oTypeData[body.id()] = person;

				}
				qDebug() << " PERSONAS = " << oTypeData.size();
				return true;
			}
		}
		return false;
	}

	bool OtoI(const RoboCompHumanTracker::PersonList &oTypeData, astra::BodyFrame &iTypeData, bool clear=false)
	{
		return false;
	}
};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
