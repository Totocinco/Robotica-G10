/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if (this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
#ifdef HIBERNATION_ENABLED
		hibernationChecker.start(500);
#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0)
		{
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
	std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

	// initializeCODE
	/////////GET PARAMS, OPEND DEVICES....////////
	// int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
	// std::string device = configLoader.get<std::string>("Device.name")

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900, 450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}

void SpecificWorker::compute()
{
	fps.print("Compute worker", 3000);

	// computeCODE

	RoboCompLidar3D::TData data;
	try
	{
		data = lidar3d_proxy->getLidarData("Lidar", 0.f, 2 * M_PI, 1);
		// qInfo() << data.points.size();
		
	}

	catch (const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
		return;
	}
    
	
	//auto [adv, rot] = StateMachine(data.points);

	try
	{
		
		//omnirobot_proxy->setSpeedBase(0.0, adv, rot);
	}
	catch (const Ice::Exception &e)
	{
		std::cerr << e.what() << "\n";
	}

	try
	{
		auto frame = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
		
	}
	catch (const Ice::Exception &e)
	{
		std::cerr << e.what() << "\n";
	}
	auto personData = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});
		RoboCompVisualElements::TObject persona = personData.objects.front();
		QRect roi;
		try
	{
		
		std::cout<<persona.bot<<" "<<persona.left<<" "<<persona.top<<" "<<persona.right<<std::endl; 
		roi = QRect(persona.left, persona.top, persona.right - persona.left, persona.bot - persona.top);


	}
	catch (const Ice::Exception &e)
	{
		std::cerr << e.what() << "\n";
	}

	// GET rgb 360
	RoboCompCamera360RGB::TImage imagen;
	try{imagen = camera360rgb_proxy->getROI(0 , 0, 0, 0, 640, 680);}catch(const Ice::Exception& e)
	{std::cout << "error en calculo de angulo" << std::endl;}
			
	
	double columnas_persona_offset = (persona.right - persona.left)/2 + persona.left - imagen.width/2 ;
	double angulo_persona = (2.0*M_PI / imagen.width) * columnas_persona_offset;

	// get distance to person 
	Eigen::Isometry3d cam_transform = Eigen::Isometry3d::Identity();
	cam_transform.pretranslate(-Eigen::Vector3d{0.f, -170.f, 1275.f});	
	std::tuple<std::vector<QPointF>, float> res;
    if(const auto res_ = project_lidar_to_camera(data.points, cam_transform,angulo_persona, imagen, roi); not res_.has_value())
	{	qWarning()<< "Error en frame";	return;}
	else res= res_.value();
	const auto & [proyected_points, depth] = res;
	draw_image(imagen, roi, proyected_points, label);
	draw_person_in_lidar(angulo_persona, depth, &viewer->scene );
	draw_lidar(data.points, &viewer->scene);

	
	// move the robot
	try
	{ 
		static PIDcontroller rotacion = PIDcontroller(3, 0, 0, 25);
		static PIDcontroller velocidad = PIDcontroller(1, 0, 0, 3000 );
		if(depth>1000)
		omnirobot_proxy->setSpeedBase(0, (velocidad.ajustar_error(depth - 1000)) * freno(-angulo_persona), rotacion.ajustar_error(-angulo_persona)); 
		else
		omnirobot_proxy->setSpeedBase(0, 0, rotacion.ajustar_error(-angulo_persona)); 
	}
	catch(const std::exception& e)
	{std::cerr << e.what() << '\n';}
	
	      
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompLidar3D::TPoints SpecificWorker::cono(const RoboCompLidar3D::TPoints& points, float minAngle, float maxAngle, float distance)
{
   // Filter points based on the specified angle range and distance
   auto filtered = points | std::views::filter([=](const auto& point)
   {
      return point.phi >= minAngle && point.phi <= maxAngle && point.distance2d > distance;   
   });

   // Materialize view to vector to match return type RoboCompLidar3D::TPoints
   return std::ranges::to<RoboCompLidar3D::TPoints>(filtered);
}


float SpecificWorker::freno(double rot)
{
	static constexpr float sigma = (-0.5 * -0.5) / std::log(0.3);
	return std::exp((rot*rot)/sigma); 
}

std::optional<std::tuple<std::vector<QPointF>, float>>
SpecificWorker::project_lidar_to_camera(
                                           const RoboCompLidar3D::TPoints& points,
                                           const Eigen::Isometry3d& cam_transform,
                                           double personAngle,
                                           const RoboCompCamera360RGB::TImage &image,
                                           const QRect &roi,
                                           double azimuth_sign,
                                           double azimuth_offset)
{


   // filter points based on the specified angle range and distance
   auto filtered_cone = cono(points, (float)personAngle - 0.3, (float)personAngle + 0.3 , 0);
   if(filtered_cone.empty()) { qWarning() << "Empty cone"; return {};};


   std::vector<QPointF> projected_points;
   projected_points.reserve(points.size());
   std::vector<float> distances;


   for (const auto& lp : filtered_cone)
   {
       // Convert Lidar point to Eigen vector
       Eigen::Vector3d p_lidar(lp.x, lp.y, lp.z);


       // Transform point to Camera Local Frame
       // P_cam = R_inv * (P_lidar - t)
       Eigen::Vector3d p_cam = cam_transform * p_lidar;


       // Equirectangular Projection
       const double r = p_cam.norm();
       if (r < 1e-9)
       {
           projected_points.emplace_back(image.width / 2.0, image.height / 2.0);
           continue;
       }


       // Azimuth (u)
       double u = ((azimuth_sign * std::atan2(p_cam.x(), p_cam.y()) + azimuth_offset) / (2.0 * M_PI) + 0.5) * image.width;
       u = std::fmod(u, static_cast<double>(image.width));
       if (u < 0.0) u += image.width;


       // Elevation (v)
       const double v = (std::asin(-p_cam.z() / r) / M_PI + 0.5) * image.height;


       // Check if the projected point is within the person's bounding box
       if (roi.contains(QPoint(u, v)))
       {
           projected_points.emplace_back(u, v);
           distances.push_back(r);
       }
   }
   // compute the minimal depth of the projected points within the bounding box
   if(distances.empty()) return {};
   float measured_depth = *std::ranges::min_element(distances);


   return std::make_tuple(projected_points, measured_depth);
}

void SpecificWorker::draw_image(const RoboCompCamera360RGB::TImage &raw, const QRect &roi, const std::vector<QPointF> &projected_points,  QLabel *label)
{
   QImage img(raw.image.data(),
              raw.width,
              raw.height,
              raw.width * 3,
              QImage::Format_RGB888);


   QSize targetSize = label->size();


   QImage resized = img.scaled(
       targetSize,
       Qt::IgnoreAspectRatio,
       Qt::SmoothTransformation
   );


   double sx = double(resized.width()) / raw.width;
   double sy = double(resized.height()) / raw.height;


   QPainter painter(&resized);


   QPen pen(Qt::red, 2);
   pen.setCosmetic(true);
   painter.setPen(pen);


   painter.scale(sx, sy);


   painter.drawRect(roi);


   // Draw projected points
   pen.setColor(Qt::blue);
   painter.setPen(pen);
   for (const auto& pt : projected_points)
       painter.drawPoint(pt);


   // Draw a vertical line at mid-width with thickness proportional to image height
   const double image_mid_x = raw.width / 2.0;
   const int vertical_guide_thickness = std::clamp<int>(static_cast<int>(raw.height / 60.0), 2, 12);
   pen.setColor(Qt::green);
   pen.setWidth(vertical_guide_thickness);
   pen.setCosmetic(false);
   painter.setPen(pen);
   painter.drawLine(QPointF(image_mid_x, 0.0), QPointF(image_mid_x, raw.height));


   // Draw a horizontal line from the closest ROI side to the vertical guide
   const double roi_mid_y = roi.top() + roi.height() / 2.0;
   const double roi_left_dist = std::abs(roi.left() - image_mid_x);
   const double roi_right_dist = std::abs(roi.right() - image_mid_x);
   const double roi_closest_x = (roi_left_dist <= roi_right_dist) ? roi.left() : roi.right();
   const double roi_guide_length = std::abs(image_mid_x - roi_closest_x);
   const int horizontal_guide_thickness = std::clamp<int>(static_cast<int>(roi_guide_length / 8.0), 2, 25);
   pen.setColor(QColor(255, 140, 0));
   pen.setWidth(horizontal_guide_thickness);
   pen.setCosmetic(false);
   painter.setPen(pen);
   painter.drawLine(QPointF(roi_closest_x, roi_mid_y), QPointF(image_mid_x, roi_mid_y));


   label->setPixmap(QPixmap::fromImage(resized));
}


void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
  static std::vector<QGraphicsItem*> draw_points;
  for (const auto &p : draw_points)
  {
     scene->removeItem(p);
     delete p;
  }
  draw_points.clear();


  const QColor color("LightGreen");
  const QPen pen(color, 10);
  //const QBrush brush(color, Qt::SolidPattern);
  for (const auto &p : points)
  {
     const auto dp = scene->addRect(-25, -25, 50, 50, pen);
     dp->setPos(p.x, p.y);
     draw_points.push_back(dp);   // add to the list of points to be deleted next time
  }
}


void SpecificWorker::draw_person_in_lidar(float person_angle, float distance, QGraphicsScene* scene)
{
   static std::vector<QGraphicsItem*> draw_points;
   for (const auto &p : draw_points)
   {
     scene->removeItem(p);
     delete p;
   }
   draw_points.clear();


   const QColor color(Qt::red);
   const QPen pen(color, 50);
  
   auto circ = scene->addEllipse(-100, -100, 200, 200, pen);
   // compute center from angle and dist
   auto cx = distance * sin(person_angle+0.1);
   auto cy = distance * cos(person_angle);
   QPointF center{cx, cy};
   circ->setPos(center);
   draw_points.push_back(circ);   


   auto line = scene->addLine(0, 0, center.x(), center.y(), pen);
   line->setZValue(-1); // Ensure the line is behind the person polygon
   draw_points.push_back(line);


   // Add text label with distance and angle
   double angle_deg = person_angle * (180.0 / M_PI);
   QString labelText = QString("Target Person\nDist: %1mm\nAng: %2°")
                           .arg(distance, 0, 'f', 2)
                           .arg(angle_deg, 0, 'f', 1);


   static QGraphicsTextItem* m_personLabel = nullptr; // Persistent label across calls
   // Create or Update the Label
   if (m_personLabel== nullptr)
   {
       // Create the item once and add it to the scene
       m_personLabel = scene->addText(labelText);
      
       // --- FIX FOR FLIPPED Y-AXIS ---
       // This cancels out the scene's scale(1, -1) transform
       m_personLabel->setTransform(QTransform().scale(1, -1));
       // -----------------------------


       // Set a visible style
       QFont font = m_personLabel->font();
       font.setPixelSize(100);
       //font.setBold(true);
       m_personLabel->setFont(font);
       m_personLabel->setDefaultTextColor(Qt::blue);
      
       // Add a background box for readability (optional but recommended)
       // Since QGraphicsTextItem doesn't have a background, we wrap it in a GraphicsItem
       // or just set a semi-transparent background via a separate rectangle if needed.
   } else     // Update existing item's text
       m_personLabel->setPlainText(labelText);
  


   // Update Position (slightly offset from the point)
   m_personLabel->setPos(center.x() + 10, center.y() - 10);
  
   // Ensure the label is always on top of other Lidar points
   m_personLabel->setZValue(100);
}



void SpecificWorker::new_target_slot(QPointF)
{
}



// std::tuple<float, float> SpecificWorker::StateMachine(const auto &points)
// {

// 	static std::chrono::time_point<std::chrono::system_clock> init;
// 	static std::chrono::time_point<std::chrono::system_clock> now;
// 	switch (state)
// 	{
// 	case State::FORWARD:
// 	{
// 		auto con = cono(points, -0.3, 0.3, 0);
// 		auto min = std::ranges::min_element(con, [](auto &a, auto &b)
// 											{ return a.distance2d < b.distance2d; });
// 		if (min != con.end() && min->distance2d > 350)
// 		{
// 			if (min->distance2d < SECURITY_THRESHOLD + 50)
// 			{
// 				state = State::TURN;
// 				// tirar un dado entre 0.4 y 3
// 				numRand = (1 + (std::rand() % 4)) * 1000;
// 				init = std::chrono::system_clock::now();
// 				return {0, 0};
// 			}
// 			return {movSpeed, 0};
// 		}
// 		return {-movSpeed, 0};
// 		break;
// 	}
// 	// controlador proporcianal
// 	case State::TURN:
// 	{

// 		auto conIZQ = cono(points, -0.35, -0.1, 0);
// 		auto conDER = cono(points, 0.1, 0.35, 0);
// 		auto minIZQ = std::ranges::min_element(conIZQ, [](auto &a, auto &b)
// 											   { return a.distance2d < b.distance2d; });
// 		auto minDER = std::ranges::min_element(conDER, [](auto &a, auto &b)
// 											   { return a.distance2d < b.distance2d; });
// 		if (minIZQ != conIZQ.end() || minDER != conDER.end())
// 		{

// 			if ((minDER->distance2d >= minIZQ->distance2d && turnLEFT) || turningLEFT)
// 			{
// 				turningLEFT = true;
// 				if (minIZQ->distance2d > 350)
// 				{
// 					now = std::chrono::high_resolution_clock::now();
// 					if (minIZQ->distance2d > SECURITY_THRESHOLD + 150 && std::chrono::duration_cast<std::chrono::milliseconds>(now - init).count() > numRand /* now - init > dado*/)
// 					{
// 						state = State::FORWARD;
// 						turningLEFT = false;
// 						return {movSpeed, 0};
// 					}
// 					float turnLEFT = -1 * rotSpeedFreeroam;
// 					return {0, turnLEFT};
// 				}
// 			}
// 			else
// 			{
// 				turnLEFT = false;
// 				if (minDER->distance2d > 350)
// 				{
// 					now = std::chrono::high_resolution_clock::now();
// 					if (minDER->distance2d > SECURITY_THRESHOLD + 150 && std::chrono::duration_cast<std::chrono::milliseconds>(now - init).count() > numRand /* now - init > dado*/)
// 					{
// 						state = State::FORWARD;
// 						turnLEFT = true;
// 						return {movSpeed, 0};
// 					}
// 					return {0, rotSpeedFreeroam};
// 				}
// 			}
// 		}
// 		if (turningLEFT)
// 		{
// 			return {0, -1 * rotSpeedFreeroam};
// 		}
// 		return {0, rotSpeedFreeroam};
// 		break;
// 	}

// 	case State::SPIRAL:
// 	{
// 		auto con = cono(points, -1, 1, 0);
// 		if (auto min = std::ranges::min_element(con, [](auto &a, auto &b)
// 												{ return a.distance2d < b.distance2d; });
// 			min != con.end() && min->distance2d > 350)
// 		{
// 			now = std::chrono::high_resolution_clock::now();
// 			printf("%f ESTO ES SPIRAL\n", min->distance2d);
// 			spiral_velocity = (spiralDiff / (2 * M_PI)) * rotSpeedSpiral * rotSpeedSpiral * ((std::chrono::duration_cast<std::chrono::milliseconds>(now - spiral_time).count() / 1000.0));
// 			if (min->distance2d < SECURITY_THRESHOLD + 100)
// 			{
// 				state = State::TURNWALL;
// 				init = std::chrono::system_clock::now();
// 				return {0, 0};
// 			}
// 			return {spiral_velocity * 1000, rotSpeedSpiral};
// 		}
// 		return {0, 0};
// 		break;
// 	}
// 	case State::TURNWALL:
// 	{

// 		auto conIZQ = cono(points, -0.6, -0.1, 1500);
// 		auto conDER = cono(points, 0.1, 0.6, 1500);
// 		auto minIZQ = std::ranges::min_element(conIZQ, [](auto &a, auto &b)
// 											   { return a.distance2d < b.distance2d; });
// 		auto minDER = std::ranges::min_element(conDER, [](auto &a, auto &b)
// 											   { return a.distance2d < b.distance2d; });
// 		if (minIZQ != conIZQ.end() || minDER != conDER.end())
// 		{
// 			printf("ESTAMOS EN TURNWALL \n");
// 			if ((minDER->distance2d >= minIZQ->distance2d && turnLEFT) || turningLEFT)
// 			{
// 				turningLEFT = true;
// 				printf("%f distancia min", minIZQ->distance2d);
// 				if (minIZQ->distance2d > 350)
// 				{
// 					now = std::chrono::high_resolution_clock::now();
// 					printf("%f distancia min", minIZQ->distance2d);
// 					if (minIZQ->distance2d > SECURITY_THRESHOLD + 250 && minIZQ->distance2d < SECURITY_THRESHOLD + 600)
// 					{
// 						if (std::chrono::duration_cast<std::chrono::milliseconds>(now - init).count() > wallTime /* now - init > dado*/)
// 						{
// 							printf("Salimos \n");
// 							state = State::FORWARD;
// 							turningLEFT = false;
// 							return {movSpeed, 0};
// 						}
// 						else
// 						{
// 							state = State::FORWARDWALL;
// 							turningLEFT = false;
// 							return {movSpeed, 0};
// 						}
// 					}
// 					float turnLEFT = -1 * ((SECURITY_THRESHOLD + 500 - minIZQ->distance2d) / 1000.0);
// 					return {0, turnLEFT};
// 				}
// 			}
// 			else
// 			{
// 				turnLEFT = false;
// 				if (minDER->distance2d > 350)
// 				{
// 					printf("%f distancia min", minDER->distance2d);
// 					now = std::chrono::high_resolution_clock::now();
// 					if (minDER->distance2d > SECURITY_THRESHOLD + 250 && minDER->distance2d < SECURITY_THRESHOLD + 600)
// 					{
// 						if (std::chrono::duration_cast<std::chrono::milliseconds>(now - init).count() > wallTime /* now - init > dado*/)
// 						{
// 							printf("Salimos \n");
// 							state = State::FORWARD;
// 							return {movSpeed, 0};
// 						}
// 						else
// 						{
// 							state = State::FORWARDWALL;
// 							return {movSpeed, 0};
// 						}
// 					}

// 					return {0, ((SECURITY_THRESHOLD + 500 - minDER->distance2d) / 1000.0)};
// 				}
// 			}
// 		}
// 		if (turningLEFT)
// 		{
// 			return {-500, -rotSpeedFreeroam};
// 		}
// 		return {-500, rotSpeedFreeroam};
// 		break;
// 	}
// 	case State::FORWARDWALL:
// 	{
// 		printf("ESTAMOS EN FORWARDWALL \n");
// 		auto con = cono(points, -0.6, 0.6, 1500);
// 		if (auto min = std::ranges::min_element(con, [](auto &a, auto &b)
// 												{ return a.distance2d < b.distance2d; });
// 			min != con.end())
// 		{
// 			printf("%f", min->distance2d);
// 			if (min->distance2d < SECURITY_THRESHOLD - 100 || min->distance2d > SECURITY_THRESHOLD + 600)
// 			{
// 				printf("%f", min->distance2d);
// 				state = State::TURNWALL;
// 				return {0, 0};
// 			}
// 		}
// 		return {movSpeed, 0};
// 		break;
// 	}
// 	}
// 	return {0, 0};
// }

////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::emergency()
{
	fps.print("Emergency worker", 3000);
	// emergencyCODE
	//
	// if (SUCCESSFUL) //The componet is safe for continue
	//   emmit goToRestore()
}

// Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	// restoreCODE
	// Restore emergency component
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// RoboCompVisualElements::TObjects this->visualelements_proxy->getVisualObjects(TObjects objects)
// RoboCompVisualElements::void this->visualelements_proxy->setVisualObjects(TObjects objects)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects
