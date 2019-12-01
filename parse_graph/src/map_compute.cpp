/**
 * Copyright (c) 2019, The Parse_graph author.
 * All rights reserved.
 *
 * Author:ybg
 * This is the node file for computing the map postier probability
 */


#include "map_compute.h"

std::map<string,std::map<string, std::map<string, float>>> relationships_p;
std::map<string, std::map<string, float>> rela_pp; 
std::map<string,float> rela_p;

std::tuple<string,string,string,float> rela_after_map_one;


void Map_Compute::Compute_Object_Map(boost::property_tree::ptree office_object,
                                    const darknet_ros_msgs::BoundingBoxes& Bound_msg,
                                    std::map<string,Vector4d>& after_object_map)
{
    // 统计同一物体出现多个概率的情况
    bool happen_flag=false;
    // record the probobility of object
    std::vector<std::map<string,float>> id_object_p;

    std::map<string,float> object_p;
    std::vector<Vector4d> object_Bbox;

    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        happen_flag=false;
        object_p.clear();
        if(!object_Bbox.empty())
        {
            for(int i=0;i<object_Bbox.size();i++)
            {
                if(Bbox.xmin == object_Bbox[i](0) && Bbox.xmax == object_Bbox[i](1) && Bbox.ymin == object_Bbox[i](2) && Bbox.ymax == object_Bbox[i](3))
                {
                    id_object_p[i].insert(std::map<string,float>::value_type(Bbox.Class,Bbox.probability));
                    cout << "happend!" << endl;
                    happen_flag=true;
                    break;
                }
               
            }
            
        }
        if(happen_flag==true) 
            continue;
        
        // 保存Bbox
        object_Bbox.push_back(Vector4d(Bbox.xmin,Bbox.xmax,Bbox.ymin,Bbox.ymax));
        // 保存类别对应的p
        object_p.insert(std::map<string,float>::value_type(Bbox.Class,Bbox.probability));
        // 保存该类别到object
        id_object_p.push_back(object_p);
    }

    // std::cout << "!!!!!!!!!!!!!!" << id_object_p.size() << endl;
    // compute probability
    for(int i=0;i<id_object_p.size();i++)
    {
        string map_name;
        float MAP=0;
        if(id_object_p[i].size()>1)
        {
            for(auto iter_ = id_object_p[i].begin();iter_ != id_object_p[i].end(); iter_++)
            {
                // cout << iter_->first << " : " << iter_->second;
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, office_object)
                {
                    if(iter_->first == v.first)
                    {
                        float office_object_p = office_object.get<float>(iter_->first);
                        
                        if(MAP < iter_->second*office_object_p)
                        {
                            MAP = iter_->second*office_object_p;
                            map_name = iter_->first;                        
                        }
                        cout << iter_->first << " : " << iter_->second << " * " << office_object_p << " = " << iter_->second*office_object_p << endl;                      
                        break;
                    }
                }   
            }
        }
        else
        {
            map_name = id_object_p[i].begin()->first;
        }

        after_object_map.insert(std::map<string,Vector4d>::value_type(map_name,object_Bbox[i]));
        // cout << i ;

        // cout << map_name << " : " << MAP;
        
        // cout << endl;
    }
}


void Map_Compute::Compute_Relationships_Map(boost::property_tree::ptree office_relationships,
                                            std::map<string,Vector9d> Support_box,
                                            std::map<string,Vector3d> On_box,
                                            std::map<string,Vector3d> object_V,
                                            std::vector<std::tuple<string,string,string,float>>& rela_after_map)
{
    // record the relationships each pair of objects and the probobility
    relationships_p.clear();

    for(auto iter = Support_box.begin();iter != Support_box.end(); iter++)
    {
        string name_support_object_ = iter->first;
        Vector9d Sbox_support_object_ = iter->second;
        rela_pp.clear(); 

        for(auto iter_ = On_box.begin();iter_ != On_box.end(); iter_++)
        {
            string name_on_object_ = iter_->first;
            Vector3d Sbox_on_object = iter_->second;
            rela_p.clear();

            // support面积支撑点   
            Vector2d p(Sbox_on_object[0],Sbox_on_object[1]);           
            Vector2d a(Sbox_support_object_[0],Sbox_support_object_[1]);           
            Vector2d b(Sbox_support_object_[2],Sbox_support_object_[3]);           
            Vector2d c(Sbox_support_object_[4],Sbox_support_object_[5]);           
            Vector2d d(Sbox_support_object_[6],Sbox_support_object_[7]);           
    
            // 在on的面积上均匀100个点进行采样，判断多少位于support的面积之内
            int Support_Sample_Sum=0;
            int Support_Sample_Count=0;                       
            for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/10.0))
                for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/10.0))
                {
                    Support_Sample_Sum++;
                    Vector2d pp(Sbox_on_object[0],Sbox_on_object[1]);                                       
                    if((pp-a).transpose()*(b-a) > 0
                    && (pp-a).transpose()*(c-a) > 0
                    && (pp-d).transpose()*(b-d) > 0
                    && (pp-d).transpose()*(c-d) > 0)
                    {
                        Support_Sample_Count++;
                        // cout << "i : " << i << " , j: " << j << "Support_Sample_Count : " << Support_Sample_Count << "object_V[name_on_object_][1] :" << object_V[name_on_object_][1] << endl;
                    }
                }
            // cout << "Support_Sample_Count : " << Support_Sample_Count << endl;
            // cout << "Support_Sample_Sum : " << Support_Sample_Sum << endl;

            // support概率函数
            float Threshold=object_V[name_on_object_][2]/2;
            float High_Error = abs(Sbox_on_object[2]-object_V[name_on_object_][2]/2 - Sbox_support_object_[8]);
            float Rela_Function=0;
            if(High_Error > Threshold)
                Rela_Function=0;
            else 
                Rela_Function=cos(M_PI*High_Error/(2*Threshold));
            // cout << "High_Error : " << High_Error << ", Threshold : " << Threshold << " ,Rela_Function : " << Rela_Function << endl;
            
            float rela_on_p=Support_Sample_Sum>0?float(Support_Sample_Count*Rela_Function/Support_Sample_Sum):0;
            cout << "Support_Sample_Count : " << Support_Sample_Count << "later : " << rela_on_p << endl;
            
            if(rela_p.count("ON") == 0 && rela_on_p>0.2)
            {
                rela_p.insert(map<string, float>::value_type("ON", rela_on_p));     
            }             
    

            // contain
            // 在on的体积上均匀1000个点进行采样，判断多少位于support的体积之内                    
            int Contain_Sample_Count=0;
            float Contain_Sample_Count_Sum =0;
            for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/10.0))
                for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/10.0))
                    for(float k = Sbox_on_object[2]-object_V[name_on_object_][2]/2 ; k < Sbox_on_object[2]+object_V[name_on_object_][2]/2 ; k = k+object_V[name_on_object_][2]/10.0)                    
                    {
                        Contain_Sample_Count_Sum++;
                        if(i > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                        && i < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                        && j > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                        && j < max(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                        && k > Sbox_support_object_[8]-object_V[name_on_object_][2]   //z > Zmin
                        && k < Sbox_support_object_[8])  //z < Zmax
                        {
                            Contain_Sample_Count++;
                        }
                    }
            // cout << "Contain_Sample_Count_Sum : " << Contain_Sample_Count_Sum << endl;                    
            cout << "contain_sample_count : " << Contain_Sample_Count << endl;
            float rela_contain_p=Contain_Sample_Count_Sum>0?float(Contain_Sample_Count/Contain_Sample_Count_Sum):0;                   
            if(rela_p.count("IN") == 0 && rela_contain_p>0.2)
            {
                rela_p.insert(map<string, float>::value_type("IN", rela_contain_p));                     
            }

                // adjoin
            //    if(contain_sample_count <100 && support_sample_count < 10 &&
            //    sqrt((Sbox_on_object[0]-Sbox_support_object_[0])*(Sbox_on_object[0]-Sbox_support_object_[0])+
            //    (Sbox_on_object[1]-Sbox_support_object_[1])*(Sbox_on_object[1]-Sbox_support_object_[1])) < 
            //    (max(object_V[name_on_object_][0],object_V[name_on_object_][1])+max(object_V[name_support_object_][0],object_V[name_support_object_][1]))*2)
            //    {
            //        float distance = sqrt((Sbox_on_object[0]-Sbox_support_object_[0])*(Sbox_on_object[0]-Sbox_support_object_[0])+
            //         (Sbox_on_object[1]-Sbox_support_object_[1])*(Sbox_on_object[1]-Sbox_support_object_[1]));

            //         cout << "distance : " << distance << endl;
            //        if(adjoin_relationships.count(name_on_object_)>0)
            //        {
            //            adjoin_relationships[name_support_object_].push_back(name_on_object_);
            //        }
            //        else
            //        {
            //             std::vector<string> adjoin_object;
            //             adjoin_object.push_back(name_on_object_);
            //             adjoin_relationships.insert(map<string, std::vector<string>>::value_type(name_support_object_, adjoin_object));
            //        }
            //    }

            
            
            // 记录与s和o有关的所有关系，以on为string
            if(rela_p.size()>1)
                cout << name_support_object_ << " and " << name_on_object_ << " rela_size: " << rela_p.size() << endl;

            if(!rela_p.empty())
                rela_pp.insert(map<string, std::map<string, float>>::value_type(name_on_object_, rela_p));
    
        }
            cout << "rela_pp.size: " << rela_pp.size() << endl;

        // 将on加入到s
        if(!rela_pp.empty())      
            relationships_p.insert(map<string, std::map<string, std::map<string, float>>>::value_type(name_support_object_, rela_pp));

    }

    // map寻优
    for(auto iter = relationships_p.begin();iter != relationships_p.end(); iter++)
    {
        string sub_name = iter->first;
        // cout << "sub_name : " << sub_name << endl;
        std::map<string, std::map<string, float>> sub_rela = iter->second;
        // cout << "sub_rela.size : " << sub_rela.size() << endl;
        
        for(auto iter_ = sub_rela.begin();iter_ != sub_rela.end(); iter_++)
        {
            string ob_name = iter_->first;
            cout << "ob_name : " << ob_name << endl;
            std::map<string, float> rela_p_all = iter_->second;
            cout << "rela_p_all.size:" << rela_p_all.size() << endl;

            // 若出现多个关系概率，使用map进行验证
            if(rela_p_all.size()>1)
            {
                string map_name;
                float MAP=0;
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, office_relationships) //object层
                {
                    if(v.first == ob_name)
                    {
                        boost::property_tree::ptree vt = v.second;
                        BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, vt)  // subject层
                        {
                            boost::property_tree::ptree vttt = vtt.second;
                            if(vtt.first == sub_name)
                            {
                                for(auto iter__ = rela_p_all.begin();iter__ != rela_p_all.end(); iter__++)
                                {
                                    string rela_all_name = iter__->first;
                                    float rela_all_p = iter__->second;
                                    BOOST_FOREACH (boost::property_tree::ptree::value_type &vtttt, vttt)  // 关系层
                                    {
                                        if(rela_all_name == vtttt.first)
                                        {
                                            if(MAP <= rela_all_p*vttt.get<float>(vtttt.first))
                                            {
                                                MAP = rela_all_p*vttt.get<float>(vtttt.first);
                                                map_name = rela_all_name;
                                            }
                                            cout << rela_all_name << " : " << rela_all_p << " * " << vttt.get<float>(vtttt.first) << " = " << MAP << endl;
                                            
                                        }
                                    }
                                }
                            }
                            
                        }
                    }
                }
                cout << "map_name : " << map_name << endl;

                rela_after_map_one = std::make_tuple(ob_name,sub_name,map_name,rela_p_all[map_name]);
                
            }
            else 
            {
                rela_after_map_one = std::make_tuple(ob_name,sub_name,
                                                    rela_p_all.begin()->first,
                                                    rela_p_all.begin()->second);
            }

            rela_after_map.push_back(rela_after_map_one);
        }

    }
    
            cout << "rela_after_map size : " << rela_after_map.size() << endl;
    
}
