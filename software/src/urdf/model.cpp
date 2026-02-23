/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "urdf/model.h"

/* we include the default parser for plain URDF files;
   other parsers are loaded via plugins (if available) */
#include <urdf_parser/urdf_parser.h>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <tinyxml.h>
#include <tinyxml2.h>

namespace urdf
{

static bool IsColladaData(const std::string & data)
{
  return data.find("<COLLADA") != std::string::npos;
}

bool Model::initFile(const std::string & filename)
{
  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open()) {
    while (xml_file.good() ) {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return Model::initString(xml_string);
  } else {
    return false;
  }
}


bool Model::initXml(TiXmlDocument * xml_doc)
{
  if (!xml_doc) {
    // ROS_ERROR("Could not parse the xml document");
    return false;
  }

  std::stringstream ss;
  ss << *xml_doc;

  return Model::initString(ss.str());
}

bool Model::initXml(TiXmlElement * robot_xml)
{
  if (!robot_xml) {
    // ROS_ERROR("Could not parse the xml element");
    return false;
  }

  std::stringstream ss;
  ss << (*robot_xml);

  return Model::initString(ss.str());
}

bool Model::initXml(const tinyxml2::XMLDocument *xml_doc)
{
  if (!xml_doc) {
    // ROS_ERROR("Could not parse the xml document");
    return false;
  }

  tinyxml2::XMLPrinter printer;
  xml_doc->Print(&printer);
  std::string str(printer.CStr());

  return Model::initString(str);
}

bool Model::initXml(const tinyxml2::XMLElement *robot_xml)
{
  if (!robot_xml) {
    // ROS_ERROR("Could not parse the xml element");
    return false;
  }

  std::stringstream ss;
  tinyxml2::XMLPrinter printer;
  robot_xml->Accept(&printer);
  ss << printer.CStr();

  return Model::initString(ss.str());
}

bool Model::initString(const std::string & xml_string)
{
  urdf::ModelInterfaceSharedPtr model;
    model = parseURDF(xml_string);

  // copy data from model into this object
  if (model) {
    this->links_ = model->links_;
    this->joints_ = model->joints_;
    this->materials_ = model->materials_;
    this->name_ = model->name_;
    this->root_link_ = model->root_link_;
    return true;
  }
  return false;
}
}  // namespace urdf
