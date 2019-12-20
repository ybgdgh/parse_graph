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

// sum probability
std::vector<std::tuple<string,string,float>> ob_sub_sum_p;
std::map<string,float> rela_odd;

bool MAP_DEBUG = true;



float adjust_add(string class_name,float class_p)
{
    float adjust_object_p=0;
    for(int i=0;i<ob_sub_sum_p.size();i++)
    {
        string ob_name;
        float ob_sum_p;
        std::tie(ob_name,std::ignore,ob_sum_p)=ob_sub_sum_p[i];
        if(ob_name == class_name && ob_sum_p>0.5)
        {
            if(rela_odd.count(class_name) == 0)
                rela_odd.insert(map<string, float>::value_type(class_name, 
                (class_p/(1-class_p)*ob_sum_p/(1-ob_sum_p))/(1+(class_p/(1-class_p)*ob_sum_p/(1-ob_sum_p)))));
            else if(rela_odd.count(class_name) > 0)
                rela_odd[class_name] = (rela_odd[class_name]/(1-rela_odd[class_name])*ob_sum_p/(1-ob_sum_p))
                                        /(1+(rela_odd[class_name]/(1-rela_odd[class_name])*ob_sum_p/(1-ob_sum_p)));
            cout << "rela_odd : " << rela_odd[class_name] << "ob_sum_p :" << ob_sum_p << endl;
            if(rela_odd[class_name]>0.9)rela_odd[class_name]=0.9;
            break;
        }
    }
    if(rela_odd.count(class_name)>0)
        adjust_object_p = rela_odd[class_name];
    else 
        adjust_object_p = class_p;
    return adjust_object_p;
}

void Map_Compute::Compute_Object_Map(
    boost::property_tree::ptree office_object,
    const darknet_ros_msgs::BoundingBoxes& Bound_msg,
    std::vector<std::tuple<string,Vector4d,float>>& after_object_map)
{
    // 统计同一物体出现多个概率的情况
    bool happen_flag=false;
    // record the probobility of object
    std::vector<std::map<string,float>> id_object_p;

    std::map<string,float> object_p;
    std::vector<Vector4d> object_Bbox;

    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        // use relationships to adjust the probability
        // cout << Bbox.Class << " before: " << Bbox.probability << endl;
        
        float class_p = adjust_add(Bbox.Class,Bbox.probability);

        // cout << Bbox.Class << " after: " << class_p << endl;
        // 清楚边缘的检测
        // if(Bbox.xmin < 10 || Bbox.xmax > 630 || Bbox.ymin < 10 || Bbox.ymax > 470)
        //     continue;


        happen_flag=false;
        object_p.clear();
        if(!object_Bbox.empty())
        {
            for(int i=0;i<object_Bbox.size();i++)
            {
                if(Bbox.xmin == object_Bbox[i](0) && Bbox.xmax == object_Bbox[i](1) && Bbox.ymin == object_Bbox[i](2) && Bbox.ymax == object_Bbox[i](3))
                {
                    id_object_p[i].insert(std::map<string,float>::value_type(Bbox.Class,Bbox.probability));
                    if(MAP_DEBUG) cout << "one object with two p happend!" << endl;
                    happen_flag=true;
                    break;
                }
                // cout << "object_Bbox " << object_Bbox[i] << endl;
               
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
        
        for(auto iter_ = id_object_p[i].begin();iter_ != id_object_p[i].end(); iter_++)
        {
            if(MAP_DEBUG) cout << "map : " << iter_->first << " : " << iter_->second << endl;
            if(id_object_p[i].size()>1)
            {
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, office_object)
                {
                    if(iter_->first == v.first)
                    {
                        float office_object_p = office_object.get<float>(iter_->first);
                        
                        if(MAP <= iter_->second*office_object_p)
                        {
                            MAP = iter_->second*office_object_p;
                            map_name = iter_->first;                        
                        }
                        if(MAP_DEBUG) cout << iter_->first << " : " << iter_->second << " * " << office_object_p << " = " << iter_->second*office_object_p << endl;                      
                        break;
                    }
                }   
            }
            else
            {
                map_name = iter_->first;
            }
        }
        if(MAP_DEBUG)  cout << "map_name: " << map_name << "p : " << id_object_p[i][map_name] << endl;

        if(id_object_p[i][map_name]>0.9)
        {
            std::tuple<string,Vector4d,float> after_object_map_one = make_tuple(map_name,object_Bbox[i],id_object_p[i][map_name]);
            after_object_map.push_back(after_object_map_one);
        }

    }
}



// local optimize
void Map_Compute::Compute_Local_Relationships_Map(
    boost::property_tree::ptree office_relationships,
    std::map<string,Vector9d> Support_box,
    std::map<string,Vector3d> On_box_local,
    std::map<string,Vector2d> object_2d_ar_pose,
    std::map<string,Vector2d> object_2d_pose,
    std::map<string,Vector3d> object_V_local,
    std::vector<std::tuple<string,string,string,float>>& rela_after_map)
{
    // record the relationships each pair of objects and the probobility
    relationships_p.clear();

    for(auto iter = Support_box.begin();iter != Support_box.end(); iter++)
    {
        string name_support_object_ = iter->first;
        if(object_2d_ar_pose.count(name_support_object_)>0)
        {
            Vector9d Sbox_support_object_ = iter->second;
            rela_pp.clear(); 

            for(auto iter_ = On_box_local.begin();iter_ != On_box_local.end(); iter_++)
            {
                string name_on_object_ = iter_->first;
                if(MAP_DEBUG) cout << "name_on_object_ : " << name_on_object_ << endl;
                if((object_2d_pose.count(name_on_object_)>0 || object_2d_ar_pose.count(name_on_object_)>0) && object_V_local[name_on_object_][2]>0.1)
                {
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
                    for(float i=Sbox_on_object[0]-object_V_local[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V_local[name_on_object_][0]/2.0;i=i+(object_V_local[name_on_object_][0]/5.0))
                        for(float j=Sbox_on_object[1]-object_V_local[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V_local[name_on_object_][1]/2.0;j=j+(object_V_local[name_on_object_][1]/5.0))
                        {
                            Support_Sample_Sum++;
                            Vector2d pp(Sbox_on_object[0],Sbox_on_object[1]);                                       
                            if((pp-a).transpose()*(b-a) > 0
                            && (pp-a).transpose()*(c-a) > 0
                            && (pp-d).transpose()*(b-d) > 0
                            && (pp-d).transpose()*(c-d) > 0)
                            {
                                Support_Sample_Count++;
                            }
                        }
                    if(MAP_DEBUG) cout << "Support_Sample_Count : " << Support_Sample_Count << endl;
                    if(MAP_DEBUG) cout << "Support_Sample_Sum : " << Support_Sample_Sum << endl;

                    // support概率函数
                    float Threshold=object_V_local[name_on_object_][2]/2;
                    float High_Error = abs(Sbox_on_object[2]-object_V_local[name_on_object_][2]/2 - Sbox_support_object_[8]);
                    float Rela_Function=0;
                    if(High_Error > Threshold)
                        Rela_Function=0;
                    else 
                        Rela_Function=cos(M_PI*High_Error/(2*Threshold));
                    if(MAP_DEBUG) cout << "High_Error : " << High_Error << ", Threshold : " << Threshold << " ,Rela_Function : " << Rela_Function << endl;
                    
                    float rela_on_p=Support_Sample_Sum>0?float(Support_Sample_Count/**Rela_Function*//Support_Sample_Sum):0;
                    if(MAP_DEBUG) cout << "Support_Sample_Count : " << Support_Sample_Count << "later : " << rela_on_p << endl;
                    
                    if(rela_p.count("ON") == 0 && rela_on_p>0.2)
                    {
                        rela_p.insert(map<string, float>::value_type("ON", rela_on_p));     
                    }             
            
                    if(MAP_DEBUG) cout << "object_V_local : " << name_on_object_ << " : " << object_V_local[name_on_object_].matrix() << endl;
                    // contain
                    // 在on的体积上均匀1000个点进行采样，判断多少位于support的体积之内                    
                    int Contain_Sample_Count=0;
                    float Contain_Sample_Count_Sum =0;
                    for(float i=Sbox_on_object[0]-object_V_local[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V_local[name_on_object_][0]/2.0;i=i+(object_V_local[name_on_object_][0]/5.0))
                        for(float j=Sbox_on_object[1]-object_V_local[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V_local[name_on_object_][1]/2.0;j=j+(object_V_local[name_on_object_][1]/5.0))
                            for(float k = Sbox_on_object[2]-object_V_local[name_on_object_][2]/2 ; k < Sbox_on_object[2]+object_V_local[name_on_object_][2]/2 ; k = k+object_V_local[name_on_object_][2]/5.0)                    
                            {
                                Contain_Sample_Count_Sum++;
                                if(i > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && i < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && j > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                                && j < max(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                                && k > Sbox_support_object_[8]-object_V_local[name_on_object_][2]   //z > Zmin
                                && k < Sbox_support_object_[8])  //z < Zmax
                                {
                                    Contain_Sample_Count++;
                                }
                            }
                    if(MAP_DEBUG) cout << "Contain_Sample_Count_Sum : " << Contain_Sample_Count_Sum << endl;                    
                    if(MAP_DEBUG) cout << "Contain_Sample_Count : " << Contain_Sample_Count << endl;
                    float rela_contain_p=Contain_Sample_Count_Sum>0?float(Contain_Sample_Count/Contain_Sample_Count_Sum):0;                   
                    if(rela_p.count("IN") == 0 && rela_contain_p>0.2)
                    {
                        rela_p.insert(map<string, float>::value_type("IN", rela_contain_p));                     
                    }

                    
                    // 记录与s和o有关的所有关系，以on为string
                    if(rela_p.size()>1)
                        cout << name_support_object_ << " and " << name_on_object_ << " rela_size: " << rela_p.size() << endl;

                    if(!rela_p.empty())
                        rela_pp.insert(map<string, std::map<string, float>>::value_type(name_on_object_, rela_p));
                
                }
                
        
            }
                if(MAP_DEBUG) cout << "rela_pp.size: " << rela_pp.size() << endl;

            // 将on加入到s
            if(!rela_pp.empty())      
                relationships_p.insert(map<string, std::map<string, std::map<string, float>>>::value_type(name_support_object_, rela_pp));

        }
        
    }

    // map寻优
    ob_sub_sum_p.clear();
    for(auto iter = relationships_p.begin();iter != relationships_p.end(); iter++)
    {
        string sub_name = iter->first;
        if(MAP_DEBUG) cout << "sub_name : " << sub_name << endl;
        std::map<string, std::map<string, float>> sub_rela = iter->second;
        if(MAP_DEBUG) cout << "sub_rela.size : " << sub_rela.size() << endl;
        
        for(auto iter_ = sub_rela.begin();iter_ != sub_rela.end(); iter_++)
        {
            string ob_name = iter_->first;
            if(MAP_DEBUG) cout << "ob_name : " << ob_name << endl;
            std::map<string, float> rela_p_all = iter_->second;
            if(MAP_DEBUG) cout << "rela_p_all.size:" << rela_p_all.size() << endl;

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
                                            if(MAP_DEBUG) cout << rela_all_name << " : " << rela_all_p << " * " << vttt.get<float>(vtttt.first) << " = " << MAP << endl;
                                            
                                        }
                                    }
                                }
                            }
                            
                        }
                    }
                }
                if(MAP_DEBUG) cout << "map_name : " << map_name << endl;

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
    
    if(MAP_DEBUG) cout << "rela_after_map size : " << rela_after_map.size() << endl;
                                        
}


// globe optimize
void Map_Compute::Compute_Globe_Relationships_Map(
    boost::property_tree::ptree office_relationships,
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
            for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/5.0))
                for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/5.0))
                {
                    Support_Sample_Sum++;
                    Vector2d pp(Sbox_on_object[0],Sbox_on_object[1]);                                       
                    if((pp-a).transpose()*(b-a) > 0
                    && (pp-a).transpose()*(c-a) > 0
                    && (pp-d).transpose()*(b-d) > 0
                    && (pp-d).transpose()*(c-d) > 0)
                    {
                        Support_Sample_Count++;
                    }
                }
            // if(MAP_DEBUG) cout << "Support_Sample_Count : " << Support_Sample_Count << endl;
            // if(MAP_DEBUG) cout << "Support_Sample_Sum : " << Support_Sample_Sum << endl;

            // support概率函数
            float Threshold=object_V[name_on_object_][2]/2;
            float High_Error = abs(Sbox_on_object[2]-object_V[name_on_object_][2]/2 - Sbox_support_object_[8]);
            float Rela_Function=0;
            if(High_Error > Threshold)
                Rela_Function=0;
            else 
                Rela_Function=cos(M_PI*High_Error/(2*Threshold));
            if(MAP_DEBUG) cout << "High_Error : " << High_Error << ", Threshold : " << Threshold << " ,Rela_Function : " << Rela_Function << endl;
            
            float rela_on_p=Support_Sample_Sum>0?float(Support_Sample_Count/**Rela_Function*//Support_Sample_Sum):0;
            if(MAP_DEBUG) cout << "Support_Sample_Count : " << Support_Sample_Count << "later : " << rela_on_p << endl;
            
            if(rela_p.count("ON") == 0 && rela_on_p>0.2)
            {
                rela_p.insert(map<string, float>::value_type("ON", rela_on_p));     
            }             
    

            // contain
            // 在on的体积上均匀1000个点进行采样，判断多少位于support的体积之内                    
            int Contain_Sample_Count=0;
            float Contain_Sample_Count_Sum =0;
            for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/5.0))
                for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/5.0))
                    for(float k = Sbox_on_object[2]-object_V[name_on_object_][2]/2 ; k < Sbox_on_object[2]+object_V[name_on_object_][2]/2 ; k = k+object_V[name_on_object_][2]/5.0)                    
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
            if(MAP_DEBUG) cout << "Contain_Sample_Count_Sum : " << Contain_Sample_Count_Sum << endl;                    
            if(MAP_DEBUG) cout << "Contain_Sample_Count : " << Contain_Sample_Count << endl;
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
            if(MAP_DEBUG) cout << "rela_pp.size: " << rela_pp.size() << endl;

        // 将on加入到s
        if(!rela_pp.empty())      
            relationships_p.insert(map<string, std::map<string, std::map<string, float>>>::value_type(name_support_object_, rela_pp));

    }

    // map寻优
    ob_sub_sum_p.clear();
    for(auto iter = relationships_p.begin();iter != relationships_p.end(); iter++)
    {
        string sub_name = iter->first;
        if(MAP_DEBUG) cout << "sub_name : " << sub_name << endl;
        std::map<string, std::map<string, float>> sub_rela = iter->second;
        if(MAP_DEBUG) cout << "sub_rela.size : " << sub_rela.size() << endl;
        
        for(auto iter_ = sub_rela.begin();iter_ != sub_rela.end(); iter_++)
        {
            string ob_name = iter_->first;
            if(MAP_DEBUG) cout << "ob_name : " << ob_name << endl;
            std::map<string, float> rela_p_all = iter_->second;
            if(MAP_DEBUG) cout << "rela_p_all.size:" << rela_p_all.size() << endl;

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
                                            if(MAP_DEBUG) cout << rela_all_name << " : " << rela_all_p << " * " << vttt.get<float>(vtttt.first) << " = " << MAP << endl;
                                            
                                        }
                                    }
                                }
                            }
                            
                        }
                    }
                }
                if(MAP_DEBUG) cout << "map_name : " << map_name << endl;

                rela_after_map_one = std::make_tuple(ob_name,sub_name,map_name,rela_p_all[map_name]);
                
            }
            else 
            {
                rela_after_map_one = std::make_tuple(ob_name,sub_name,
                                                    rela_p_all.begin()->first,
                                                    rela_p_all.begin()->second);
            }

            rela_after_map.push_back(rela_after_map_one);

            // record the sum probability
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
                            float sum_p = vttt.get<float>("sum");
                            ob_sub_sum_p.push_back(make_tuple(ob_name,sub_name,sum_p));
                            cout << "sum p : " << ob_name << "," << sub_name << " , " << sum_p << endl;
                        }
                        
                    }
                }
            }
        }

    }
    
    if(MAP_DEBUG) cout << "rela_after_map size : " << rela_after_map.size() << endl;
    
}
