#pragma once

#include <string>

struct SwerveModuleName {
 public:
  SwerveModuleName(std::string front_back, std::string left_right)
    : m_front_back{front_back}, m_left_right{left_right} {

    if (front_back == "f") {
      if (left_right == "l") {
        m_abbr_upper = "FL";
        m_abbr_lower = "fl";
        m_camel = "frontLeft";
        m_title = "Front Left";
      } else if (left_right == "r") {
        m_abbr_upper = "FR";
        m_abbr_lower = "fr";
        m_camel = "frontRight";
        m_title = "Front Right";
      } else {
        throw;
      }
    } else if (front_back == "b") {
      if (left_right == "l") {
        m_abbr_upper = "BL";
        m_abbr_lower = "bl";
        m_camel = "backLeft";
        m_title = "Back Left";
      } else if (left_right == "r") {
        m_abbr_upper = "BR";
        m_abbr_lower = "bR";
        m_camel = "backRight";
        m_title = "Back Right";
      }
    } else {
      throw;
    }
  }

  std::string GetAbbrUpper() const {
    return m_abbr_upper;
  }

  std::string GetAbbrLower() const {
    return m_abbr_lower;
  }

  std::string GetCamelCase() const {
    return m_camel + "Module";
  }

  std::string GetFullTitle() const {
    return m_title + " Module";
  }

  std::string GetTitleCase() const {
    return m_title;
  }

 private:
  
  std::string m_front_back;
  std::string m_left_right;

  std::string m_abbr_upper;
  std::string m_abbr_lower;
  
  std::string m_camel;

  std::string m_title;
};