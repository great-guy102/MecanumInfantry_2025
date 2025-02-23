/**
 * @file      rfr_pkg_0x0301_inter_graphics.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All
 * Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <string>
#include <cstring>

#include "rfr_pkg_0x0301_inter_among_robots.hpp"
#include "system.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/

enum class DeleteOperation : uint8_t {
  kNone = 0x00,   ///< 空操作
  kLayer = 0x01,  ///< 删除图层
  kAll = 0x02,    ///< 删除所有图形
};

enum class GraphicLayer : uint8_t {
  k0 = 0x00,  ///< 图层 0
  k1 = 0x01,  ///< 图层 1
  k2 = 0x02,  ///< 图层 2
  k3 = 0x03,  ///< 图层 3
  k4 = 0x04,  ///< 图层 4
  k5 = 0x05,  ///< 图层 5
  k6 = 0x06,  ///< 图层 6
  k7 = 0x07,  ///< 图层 7
  k8 = 0x08,  ///< 图层 8
  k9 = 0x09,  ///< 图层 9
};

enum class GraphicOperation : uint8_t {
  kNone = 0x00,    ///< 空操作
  kAdd = 0x01,     ///< 添加图形
  kModify = 0x02,  ///< 修改图形
  kDelete = 0x03,  ///< 删除图形
};

enum class GraphicType : uint8_t {
  kStraightLine = 0x00,  ///< 直线
  kRectangle = 0x01,     ///< 矩形
  kCircle = 0x02,        ///< 圆形
  kEllipse = 0x03,       ///< 椭圆
  kArc = 0x04,           ///< 弧线
  kFloating = 0x05,      ///< 浮点数
  kInteger = 0x06,       ///< 整数
  kString = 0x07,        ///< 字符串
  kUnKnown,              ///< 未知
};

enum class GraphicColor : uint8_t {
  kRedOrBlue = 0x00,  ///< 主色，红/蓝（己方颜色）
  kYellow = 0x01,     ///< 黄色
  kGreen = 0x02,      ///< 绿色
  kOrange = 0x03,     ///< 橙色
  kPurple = 0x04,     ///< 紫色
  kPink = 0x05,       ///< 粉色
  kCyan = 0x06,       ///< 青色
  kBlack = 0x07,      ///< 黑色
  kWhite = 0x08,      ///< 白色
};

const uint16_t kMaxPosX = 1920u;
const uint16_t kMaxPosY = 1080u;
const size_t kMaxStringLength = 30u;

enum GraphicDataMask : uint32_t {
  kGraphicDataMaskOperateType = (0x01 << 3) - 1,
  kGraphicDataMaskFigureType = (0x01 << 3) - 1,
  kGraphicDataMaskLayer = (0x01 << 4) - 1,
  kGraphicDataMaskColor = (0x01 << 4) - 1,
  kGraphicDataMaskDetailsA = (0x01 << 9) - 1,
  kGraphicDataMaskDetailsB = (0x01 << 9) - 1,
  kGraphicDataMaskLineWidth = (0x01 << 10) - 1,
  kGraphicDataMaskStartX = (0x01 << 11) - 1,
  kGraphicDataMaskStartY = (0x01 << 11) - 1,
  kGraphicDataMaskDetailsC = (0x01 << 10) - 1,
  kGraphicDataMaskDetailsD = (0x01 << 11) - 1,
  kGraphicDataMaskDetailsE = (0x01 << 11) - 1,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef uint16_t Pixel;  ///< 像素
typedef uint16_t Ang;    ///< 角度

struct __REFEREE_PACKED GraphicDeleteData {
  uint8_t delete_operation;
  uint8_t layer;
};

struct __REFEREE_PACKED GraphicData {
  uint8_t figure_name[3];
  uint32_t operate_tpye : 3;
  uint32_t figure_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t details_a : 9;
  uint32_t details_b : 9;
  uint32_t line_width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t details_c : 10;
  uint32_t details_d : 11;
  uint32_t details_e : 11;
};

class Graphic : public MemMgr
{
 public:
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicType Type;
  typedef GraphicColor Color;
  typedef GraphicData Data;

  Graphic(void) = default;

  Graphic(const uint8_t name[3], Operation operation, Layer layer, Color color)
  {
    setName(name);
    setOperation(operation);
    setLayer(layer);
    setColor(color);
  }

  void setName(const uint8_t name[3]) { memcpy(data_.figure_name, name, 3); }
  void setOperation(Operation operation)
  {
    data_.operate_tpye =
        (uint32_t)operation & (uint32_t)kGraphicDataMaskOperateType;
  }
  void setLayer(Layer layer)
  {
    data_.layer = (uint32_t)layer & (uint32_t)kGraphicDataMaskLayer;
  }
  void setColor(Color color)
  {
    data_.color = (uint32_t)color & (uint32_t)kGraphicDataMaskColor;
  }
  void setLineWidth(Pixel line_width)
  {
    data_.line_width =
        (uint32_t)line_width & (uint32_t)kGraphicDataMaskLineWidth;
  }

  const uint8_t *getName(void) const { return data_.figure_name; }
  Operation getOperation(void) const { return (Operation)data_.operate_tpye; }
  Layer getLayer(void) const { return (Layer)data_.layer; }
  Color getColor(void) const { return (Color)data_.color; }
  Pixel getLineWidth(void) const { return data_.line_width; }

  const Data &getData(void) const { return data_; }

 protected:
  void setGraphicType(const Type &type)
  {
    data_.figure_tpye = (uint32_t)type & (uint32_t)kGraphicDataMaskFigureType;
  }

  Data data_ = {
      .figure_name = {0u, 0u, 0u},
      .operate_tpye = (uint32_t)Operation::kNone,
      .figure_tpye = (uint32_t)Type::kStraightLine,
      .layer = (uint32_t)Layer::k0,
      .color = (uint32_t)Color::kRedOrBlue,
      .details_a = 0,
      .details_b = 0,
      .line_width = 0,
      .start_x = 0,
      .start_y = 0,
      .details_c = 0,
      .details_d = 0,
      .details_e = 0,
  };
};

class StraightLine : public Graphic
{
 public:
  StraightLine(void) { setGraphicType(kGraphicType); }
  StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }
  StraightLine(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }
  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setEndPos(Pixel x, Pixel y)
  {
    setEndPosX(x);
    setEndPosY(y);
  }
  void setEndPosX(Pixel x) { data_.details_d = x & kGraphicDataMaskDetailsD; }
  void setEndPosY(Pixel y) { data_.details_e = y & kGraphicDataMaskDetailsE; }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getEndPosX(void) const { return data_.details_d; }
  Pixel getEndPosY(void) const { return data_.details_e; }

  static constexpr Type kGraphicType = Type::kStraightLine;  ///< 图形类型
};
class Rectangle : public Graphic
{
 public:
  Rectangle(void) { setGraphicType(kGraphicType); }
  Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }
  Rectangle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel end_x, Pixel end_y, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setEndPos(end_x, end_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }
  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setEndPos(Pixel x, Pixel y)
  {
    setEndPosX(x);
    setEndPosY(y);
  }
  void setEndPosX(Pixel x) { data_.details_d = x & kGraphicDataMaskDetailsD; }
  void setEndPosY(Pixel y) { data_.details_e = y & kGraphicDataMaskDetailsE; }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getEndPosX(void) const { return data_.details_d; }
  Pixel getEndPosY(void) const { return data_.details_e; }

  static constexpr Type kGraphicType = Type::kRectangle;  ///< 图形类型
};
class Circle : public Graphic
{
 public:
  Circle(void) { setGraphicType(kGraphicType); }
  Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }
  Circle(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setCenterPosX(Pixel x) { data_.start_x = x; }
  void setCenterPosY(Pixel y) { data_.start_y = y; }
  void setRadius(Pixel r) { data_.details_c = r & kGraphicDataMaskDetailsC; }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadius(void) const { return data_.details_c; }

  static constexpr Type kGraphicType = Type::kCircle;  ///< 图形类型
};
class Ellipse : public Graphic
{
 public:
  Ellipse(void) { setGraphicType(kGraphicType); }
  Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }
  Ellipse(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPosX(Pixel x) { data_.start_x = x & kGraphicDataMaskStartX; }
  void setCenterPosY(Pixel y) { data_.start_y = y & kGraphicDataMaskStartY; }
  void setRadiusX(Pixel radius_x)
  {
    data_.details_d = radius_x & kGraphicDataMaskDetailsD;
  }
  void setRadiusY(Pixel radius_y)
  {
    data_.details_e = radius_y & kGraphicDataMaskDetailsE;
  }
  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setRadius(Pixel radius_x, Pixel radius_y)
  {
    setRadiusX(radius_x);
    setRadiusY(radius_y);
  }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadiusX(void) const { return data_.details_d; }
  Pixel getRadiusY(void) const { return data_.details_e; }

  static constexpr Type kGraphicType = Type::kEllipse;  ///< 图形类型
};
class Arc : public Graphic
{
 public:
  Arc(void) { setGraphicType(kGraphicType); }
  Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Ang start_ang, Ang end_ang)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setAng(start_ang, end_ang);
    setGraphicType(kGraphicType);
    setLineWidth(1);
  }
  Arc(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel center_x, Pixel center_y, Pixel radius_x, Pixel radius_y,
      Ang start_ang, Ang end_ang, Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setCenterPos(center_x, center_y);
    setRadius(radius_x, radius_y);
    setAng(start_ang, end_ang);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setCenterPosX(Pixel x) { data_.start_x = x & kGraphicDataMaskStartX; }
  void setCenterPosY(Pixel y) { data_.start_y = y & kGraphicDataMaskStartY; }
  void setRadiusX(Pixel radius_x)
  {
    data_.details_d = radius_x & kGraphicDataMaskDetailsD;
  }
  void setRadiusY(Pixel radius_y)
  {
    data_.details_e = radius_y & kGraphicDataMaskDetailsE;
  }
  void setStartAng(Ang start_ang)
  {
    data_.details_a = start_ang & kGraphicDataMaskDetailsA;
  }
  void setEndAng(Ang end_ang)
  {
    data_.details_b = end_ang & kGraphicDataMaskDetailsB;
  }
  void setCenterPos(Pixel x, Pixel y)
  {
    setCenterPosX(x);
    setCenterPosY(y);
  }
  void setRadius(Pixel radius_x, Pixel radius_y)
  {
    setRadiusX(radius_x);
    setRadiusY(radius_y);
  }
  void setAng(Ang start_ang, Ang end_ang)
  {
    setStartAng(start_ang);
    setEndAng(end_ang);
  }

  Pixel getCenterPosX(void) const { return data_.start_x; }
  Pixel getCenterPosY(void) const { return data_.start_y; }
  Pixel getRadiusX(void) const { return data_.details_d; }
  Pixel getRadiusY(void) const { return data_.details_e; }
  Ang getStartAng(void) const { return data_.details_a; }
  Ang getEndAng(void) const { return data_.details_b; }

  static constexpr Type kGraphicType = Type::kArc;  ///< 图形类型
};
class FloatingNumber : public Graphic
{
 public:
  FloatingNumber(void) { setGraphicType(kGraphicType); }
  FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, float value)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }
  FloatingNumber(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, float value,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setFontSize(Pixel font_size) { data_.details_a = font_size; }
  void setDisplayValue(float value)
  {
    int32_t encode_value = (int32_t)(value * 1000);
    uint32_t *encode_value_ptr = (uint32_t *)(&encode_value);
    data_.details_c = (*encode_value_ptr) & kGraphicDataMaskDetailsC;
    data_.details_d = ((*encode_value_ptr) >> 10) & kGraphicDataMaskDetailsD;
    data_.details_e = ((*encode_value_ptr) >> 21) & kGraphicDataMaskDetailsE;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  float getDisplayValue(void) const { return getEncodeValue() / 1000; }
  int32_t getEncodeValue(void) const
  {
    return (int32_t)(data_.details_c << 22 | data_.details_d << 11 |
                     data_.details_e);
  }

  static constexpr Type kGraphicType = Type::kFloating;  ///< 图形类型
};
class Integer : public Graphic
{
 public:
  Integer(void) { setGraphicType(kGraphicType); }
  Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, int32_t value)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }
  Integer(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, int32_t value,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setDisplayValue(value);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x; }
  void setStartPosY(Pixel y) { data_.start_y = y; }
  void setFontSize(Pixel font_size) { data_.details_a = font_size; }
  void setDisplayValue(int32_t value)
  {
    int32_t encode_value = (int32_t)(value);
    uint32_t *encode_value_ptr = (uint32_t *)(&encode_value);
    data_.details_c = (*encode_value_ptr) & kGraphicDataMaskDetailsC;
    data_.details_d = ((*encode_value_ptr) >> 10) & kGraphicDataMaskDetailsD;
    data_.details_e = ((*encode_value_ptr) >> 21) & kGraphicDataMaskDetailsE;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  float getDisplayValue(void) const { return getEncodeValue(); }
  int32_t getEncodeValue(void) const
  {
    return (int32_t)(data_.details_c << 22 | data_.details_d << 11 |
                     data_.details_e);
  }

  static constexpr Type kGraphicType = Type::kInteger;  ///< 图形类型
};
class String : public Graphic
{
 public:
  String(void) { setGraphicType(kGraphicType); }
  String(
      const uint8_t name[3], Operation operation, Layer layer, Color color)
      : Graphic(name, operation, layer, color)
  {
    setGraphicType(kGraphicType);
  }
  String(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, size_t string_length)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setStringlength(string_length);
    setGraphicType(kGraphicType);
    setLineWidth(font_size < 10 ? 1 : font_size / 10);
  }
  String(
      const uint8_t name[3], Operation operation, Layer layer, Color color,
      Pixel start_x, Pixel start_y, Pixel font_size, size_t string_length,
      Pixel line_width)
      : Graphic(name, operation, layer, color)
  {
    setStartPos(start_x, start_y);
    setFontSize(font_size);
    setStringlength(string_length);
    setLineWidth(line_width);
    setGraphicType(kGraphicType);
  }

  void setStartPosX(Pixel x) { data_.start_x = x & kGraphicDataMaskStartX; }
  void setStartPosY(Pixel y) { data_.start_y = y & kGraphicDataMaskStartY; }
  void setFontSize(Pixel font_size)
  {
    data_.details_a = font_size & kGraphicDataMaskDetailsA;
  }
  void setStringlength(size_t length)
  {
    data_.details_b =
        (length <= kMaxStringLength ? length : kMaxStringLength) &
        kGraphicDataMaskDetailsB;
  }
  void setStartPos(Pixel x, Pixel y)
  {
    setStartPosX(x);
    setStartPosY(y);
  }

  Pixel getStartPosX(void) const { return data_.start_x; }
  Pixel getStartPosY(void) const { return data_.start_y; }
  Pixel getFontSize(void) const { return data_.details_a; }
  size_t getLength(void) const
  {
    return data_.details_b <= kMaxStringLength
               ? data_.details_b
               : kMaxStringLength;
  }

  static constexpr Type kGraphicType = Type::kString;  ///< 图形类型
};

class InterGraphicPackage : public InterAmongRobotsPackage
{
 public:
  virtual bool setSenderId(RfrId id) override
  {
    if (checkSenderId(id)) {
      sender_id_ = id;
      receiver_id_ = ids::RobotId2ClientId(id);
      return true;
    } else {
      return false;
    }
  }

  virtual bool setReceiverId(RfrId id) override
  {
    if (checkReceiverId(id)) {
      receiver_id_ = id;
      sender_id_ = ids::ClientId2RobotId(id);
      return true;
    } else {
      return false;
    }
  }

 protected:
  virtual bool checkSenderId(RfrId id) const override
  {
    return ids::IsRobotId(id);
  }

  virtual bool checkReceiverId(RfrId id) const override
  {
    return ids::IsClientId(id);
  }
};

class InterGraphicDeletePackage : public InterGraphicPackage
{
 public:
  typedef GraphicDeleteData Data;
  typedef DeleteOperation Operation;
  typedef GraphicLayer Layer;

  virtual CmdId getInterCmdId(void) const override { return 0x0100; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data);
  }

  void setDeleteOperation(Operation operation)
  {
    data_.delete_operation = (uint8_t)operation;
  }
  void setLayer(Layer layer) { data_.layer = (uint8_t)layer; }

  Operation getDeleteOperation(void) const
  {
    return (Operation)data_.delete_operation;
  }
  Layer getLayer(void) const { return (Layer)data_.layer; }

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, getInterDataLength());
  }

  Data data_ = {
      .delete_operation = (uint8_t)DeleteOperation::kAll,
      .layer = (uint8_t)Layer::k0,
  };
};

class InterGraphic1Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0101; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data);
  }

  void setStraightLine(const StraightLine &graphic)
  {
    data_ = graphic.getData();
  }
  void setRectangle(const Rectangle &graphic) { data_ = graphic.getData(); }
  void setCircle(const Circle &graphic) { data_ = graphic.getData(); }
  void setEllipse(const Ellipse &graphic) { data_ = graphic.getData(); }
  void setArc(const Arc &graphic) { data_ = graphic.getData(); }
  void setFloatingNumber(const FloatingNumber &graphic)
  {
    data_ = graphic.getData();
  }
  void setInteger(const Integer &graphic) { data_ = graphic.getData(); }
  void setGraphic(const Graphic &graphic) { data_ = graphic.getData(); }

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, sizeof(Data));
  }

  Data data_ = {0};
};

class InterGraphic2Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0102; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setRectangleAt(const Rectangle &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setCircleAt(const Circle &graphic, uint8_t idx)
  {
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
    data_[idx] = graphic.getData();
  }
  void setEllipseAt(const Ellipse &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setArcAt(const Arc &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setIntegerAt(const Integer &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setGraphicAt(const Graphic &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }

  static constexpr size_t kMaxNumGraphicData_ = 2;  ///< 最大图形数据数量

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }

  Data data_[kMaxNumGraphicData_] = {0};
};

class InterGraphic5Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0103; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setRectangleAt(const Rectangle &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setCircleAt(const Circle &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setEllipseAt(const Ellipse &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setArcAt(const Arc &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setIntegerAt(const Integer &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setGraphicAt(const Graphic &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }

  static constexpr size_t kMaxNumGraphicData_ = 5;  ///< 最大图形数据数量

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }

  Data data_[kMaxNumGraphicData_] = {0};
};

class InterGraphic7Package : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0104; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) * kMaxNumGraphicData_;
  }

  void setStraightLineAt(const StraightLine &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setRectangleAt(const Rectangle &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setCircleAt(const Circle &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setEllipseAt(const Ellipse &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setArcAt(const Arc &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setFloatingNumberAt(const FloatingNumber &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setIntegerAt(const Integer &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }
  void setGraphicAt(const Graphic &graphic, uint8_t idx)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(
        idx < kMaxNumGraphicData_, "Index should be less than %d, but got %d.",
        kMaxNumGraphicData_, idx);
#pragma endregion
    data_[idx] = graphic.getData();
  }

  static constexpr size_t kMaxNumGraphicData_ = 7;  ///< 最大图形数据数量

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    for (uint8_t i = 0; i < kMaxNumGraphicData_; i++) {
      memcpy(data + i * sizeof(Data), &data_[i], sizeof(Data));
    }
  }

  Data data_[kMaxNumGraphicData_] = {0};
};

class InterGraphicStringPackage : public InterGraphicPackage
{
 public:
  typedef GraphicData Data;
  typedef GraphicType Type;
  typedef GraphicOperation Operation;
  typedef GraphicLayer Layer;
  typedef GraphicColor Color;

  virtual CmdId getInterCmdId(void) const override { return 0x0110; }
  virtual DataLength getInterDataLength(void) const override
  {
    return sizeof(Data) + kMaxStringLength;
  }

  void setStrintg(const String &graphic, const std::string &characters)
  {
    data_ = graphic.getData();
    memset(characters_, 0, sizeof(characters_));
    memcpy(characters_, characters.c_str(),
           characters.size() > kMaxStringLength
               ? kMaxStringLength
               : characters.size());
  }
  void setString(const String &graphic, const char *characters, size_t len)
  {
    data_ = graphic.getData();
    memset(characters_, 0, sizeof(characters_));
    memcpy(characters_, characters,
           len > kMaxStringLength ? kMaxStringLength : len);
  }

 protected:
  virtual void encodeInterData(uint8_t *data) override
  {
    memcpy(data, &data_, sizeof(Data));
    memset(data + sizeof(Data), 0, sizeof(characters_));
    memcpy(data + sizeof(Data), characters_, sizeof(characters_));
  }

  Data data_ = {0};
  uint8_t characters_[30] = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_GRAPHICS_HPP_ */
