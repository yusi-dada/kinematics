#include <urdf/urdf.h>

namespace urdf_func
{
    
template<typename T>
void getGeometry(T dat)
{
	auto geometry = dat->geometry;		
	switch (geometry->type)
	{
		case urdf::Geometry::SPHERE:
		{
			std::shared_ptr<urdf::Sphere> obj = urdf::static_pointer_cast<urdf::Sphere>(geometry);
/*
			marker.type = visualization_msgs::Marker::SPHERE;
			geometry_msgs::Vector3 s;
			s.x = s.y = s.z = 2.0 * obj->radius;	// radius -> diameter
			marker.scale = s;
*/
			break;
		}
		
		case urdf::Geometry::BOX:
		{
			std::shared_ptr<urdf::Box> obj = urdf::static_pointer_cast<urdf::Box>(geometry);
/*
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale = cvtVector3(obj->dim);
*/
			break;
		}
		
		case urdf::Geometry::CYLINDER:
		{
			std::shared_ptr<urdf::Cylinder> obj = urdf::static_pointer_cast<urdf::Cylinder>(geometry);
/*
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.scale.x = obj->radius;
			marker.scale.y = obj->radius;
			marker.scale.z = obj->length;
*/
			break;
		}
		
		case urdf::Geometry::MESH:
		{
			std::shared_ptr<urdf::Mesh> obj = urdf::static_pointer_cast<urdf::Mesh>(geometry);
/*
			marker.type = visualization_msgs::Marker::MESH_RESOURCE;
			marker.mesh_resource = obj->filename;
			marker.scale = cvtVector3(obj->scale);
*/
			break;
		}
	}
	// 原点情報
//	marker.pose = cvtPose(dat->origin);

//	return(marker);
}



bool getModel(std::string xml)
{
	urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml);
	if(!model)
	{
        std::cerr << "Failed to parse urdf file" << std::endl;
        return false;
	}

    // ルート取得
    urdf::LinkConstSharedPtr root = model->getRoot();

    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);

    links[0]->visual->geometry;
    links[0]->collision->geometry;
}

}


