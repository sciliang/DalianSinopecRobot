#ifndef READXML_HPP
#define READXML_HPP
#include <string>
#include <iostream>
using namespace std;
#include <boost/lexical_cast.hpp>
#include "tinyxml2.h"
#include "glog/logging.h"

/*XML解析类，将tinyxml库进一步封装*/
class XMLParser
{
public:
    /**
        * @brief:构造函数
        * @param xmlFileName       [in]:包含路径的xml文件名，
        *                 默认是当前路径下的"configuration.xml"文件
        */
    XMLParser(string xmlFileName="configuration.xml")
    {
        //如果加载配置文件时出现错误，可以直接用浏览器打开xml文件确定是否是文件本身语法错误
        CHECK(!doc.LoadFile(xmlFileName.c_str()))<<"configuration file not exist or something wrong with xml file!";
        root=doc.RootElement();
    }

    /**
        * @brief:读取xml文件某键值下属性，返回第一个符合属性的值
        * @param keyName        [in]:键名
        *                 keyProperty   [in]:属性名
        *                 value               [out]:键值
        *                 strGroupID     [in]:组编号
        * @return
        *                   -1-failture
        *                   0-success
        */
    template <class T>
    int getValue(string keyName,string keyProperty,T &value,vector<string> vecAttribute)
    {
        tinyxml2::XMLElement* pXMLEle=root->FirstChildElement(keyName.c_str())->FirstChildElement(keyProperty.c_str());
        CHECK((vecAttribute.size()%2)==0)<<"attribute must be pairs,key name:"<<keyName<<" key property:"<<keyProperty;
        while(true)
        {
            int i=0;
            for(;i<vecAttribute.size();i=i+2)
            {
                CHECK(pXMLEle!=NULL)<<"no key name:"<<keyName<<" property:"<<keyProperty<<"found";
                if(strcmp(pXMLEle->Attribute(vecAttribute[i].c_str()),vecAttribute[i+1].c_str()))
                {
                    ///属性值不相同,跳出for循环,检测下一个
                    break;
                }
            }
            ///属性值相同,已找到目标值,跳出while循环
            if(i>=vecAttribute.size())
                break;
            pXMLEle=pXMLEle->NextSiblingElement(keyProperty.c_str());
        }

        const char* pString=pXMLEle->GetText();
        if(pString!=nullptr)
        {
            value=boost::lexical_cast<T>(pString);
            return 0;
        }
        LOG(ERROR)<<"key:"<<keyName<<"  property:"<<keyProperty<<" does not found";
        return -1;
    }

    /**
        * @brief:读取xml文件某键值下属性，返回第一个符合属性的值
        * @param keyName        [in]:键名
        *                 keyProperty   [in]:属性名
        *                 value               [out]:键值
        *                 strGroupID     [in]:组编号
        * @return
        *                   -1-failture
        *                   0-success
        */
    template <class T>
    int getValue(string keyName,string keyProperty,T &value)
    {
        tinyxml2::XMLElement* pXMLEle=root->FirstChildElement(keyName.c_str())->FirstChildElement(keyProperty.c_str());

        const char* pString=pXMLEle->GetText();
        if(pString!=nullptr)
        {
            value=boost::lexical_cast<T>(pString);
            return 0;
        }
        LOG(ERROR)<<"key:"<<keyName<<"  property:"<<keyProperty<<" does not found";
        return -1;
    }

    /**
        * @brief:读取xml文件某键值下属性，返回第所有符合属性的值
        * @param keyName        [in]:键名
        *                 keyProperty   [in]:属性名
        *                 value               [out]:键值
        *                 id                     [in]:组编号
        * @return
        *                   -1-failture
        *                   0-success
        */
    template <class T>
    int getAllValue(string keyName,string keyProperty,vector<T> &vecValue,vector<string> vecAttribute)
    {
        tinyxml2::XMLElement* element=root->FirstChildElement(keyName.c_str())->FirstChildElement(keyProperty.c_str());
        CHECK((vecAttribute.size()%2)==0)<<"attribute must be pairs,key name:"<<keyName<<" key property:"<<keyProperty;
        while(element!=nullptr)
        {
            int i=0;
            for(;i<vecAttribute.size();i=i+2)
            {
                if(strcmp(element->Attribute(vecAttribute[i].c_str()),vecAttribute[i+1].c_str()))
                {
                    ///属性值不相同,跳出for循环,检测下一个
                    break;
                }
            }
            ///属性值相同,已找到目标值,跳出while循环
            if(i>=vecAttribute.size())
            {
                T value=boost::lexical_cast<T>(element->GetText());
                vecValue.push_back(value);
            }
            element=element->NextSiblingElement(keyProperty.c_str());
        }
    }

    template <class T>
    int getAllValue(string keyName,string keyProperty,vector<T> &vecValue)
    {
        tinyxml2::XMLElement* element=root->FirstChildElement(keyName.c_str())->FirstChildElement(keyProperty.c_str());
        while(element!=nullptr)
        {
            T value=boost::lexical_cast<T>(element->GetText());
            vecValue.push_back(value);
            element=element->NextSiblingElement(keyProperty.c_str());
        }
    }

private:
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root;
};
#endif // READXML_HPP

