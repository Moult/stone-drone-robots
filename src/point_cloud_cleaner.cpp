#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <MathGeoLib.h>

#include <tr1/functional>

#include <ply.hpp>

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

// Disclaimer: I don't know C++

using namespace std::tr1::placeholders;

class Repository
{
    public:
        static bool is_loading_boundary;
        static int current_face_index;
        static ply::float32 current_vertex[3];
        static int current_face[3];
};

bool Repository::is_loading_boundary = true;
int Repository::current_face_index = 0;
float Repository::current_vertex[3] = {0, 0, 0};
int Repository::current_face[3] = {0, 0, 0};

class BoundaryChecker
{
    public:
        static Polyhedron polyhedron;
        static void add_vertex_from_repository();
        static void add_face_from_repository();
};

Polyhedron BoundaryChecker::polyhedron;

void BoundaryChecker::add_vertex_from_repository()
{
    BoundaryChecker::polyhedron.v.push_back(
        POINT_VEC(
            Repository::current_vertex[0],
            Repository::current_vertex[1],
            Repository::current_vertex[2]
        )
    );
}

void BoundaryChecker::add_face_from_repository()
{
    Polyhedron::Face face;
    face.v.insert(face.v.end(), Repository::current_face, Repository::current_face+3);
    BoundaryChecker::polyhedron.f.push_back(face);
}

class ply_to_ply_converter
{
public:
  typedef int format_type;
  enum format {
    same_format,
    ascii_format,
    binary_format,
    binary_big_endian_format,
    binary_little_endian_format
  };
  ply_to_ply_converter(format_type format) : format_(format) {}
  bool load_boundary(std::istream& bstream, std::ostream& ostream);
  bool convert(std::istream& istream, std::ostream& ostream);
private:
  void info_callback(const std::string& filename, std::size_t line_number, const std::string& message);
  void warning_callback(const std::string& filename, std::size_t line_number, const std::string& message);
  void error_callback(const std::string& filename, std::size_t line_number, const std::string& message);
  void magic_callback();
  void format_callback(ply::format_type format, const std::string& version);
  void element_begin_callback();
  void element_end_callback();
  std::tr1::tuple<std::tr1::function<void()>, std::tr1::function<void()> > element_definition_callback(const std::string& element_name, std::size_t count);
  template <typename ScalarType> void x_property_callback(ScalarType scalar);
  template <typename ScalarType> void y_property_callback(ScalarType scalar);
  template <typename ScalarType> void z_property_callback(ScalarType scalar);
  template <typename ScalarType> void scalar_property_callback(ScalarType scalar);
  template <typename ScalarType> std::tr1::function<void (ScalarType)> scalar_property_definition_callback(const std::string& element_name, const std::string& property_name);
  template <typename SizeType, typename ScalarType> void list_property_begin_callback(SizeType size);
  template <typename SizeType, typename ScalarType> void list_property_face_callback(ScalarType scalar);
  template <typename SizeType, typename ScalarType> void list_property_element_callback(ScalarType scalar);
  template <typename SizeType, typename ScalarType> void list_property_end_callback();
  template <typename SizeType, typename ScalarType> std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> > list_property_definition_callback(const std::string& element_name, const std::string& property_name);
  void comment_callback(const std::string& comment);
  void obj_info_callback(const std::string& obj_info);
  bool end_header_callback();
  format_type format_;
  ply::format_type input_format_, output_format_;
  bool bol_;
  std::ostream* ostream_;
};

void ply_to_ply_converter::info_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "info: " << message << std::endl;
}

void ply_to_ply_converter::warning_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "warning: " << message << std::endl;
}

void ply_to_ply_converter::error_callback(const std::string& filename, std::size_t line_number, const std::string& message)
{
  std::cerr << filename << ":" << line_number << ": " << "error: " << message << std::endl;
}

void ply_to_ply_converter::magic_callback()
{
  (*ostream_) << "ply" << "\n";
}

void ply_to_ply_converter::format_callback(ply::format_type format, const std::string& version)
{
  input_format_ = format;

  switch (format_) {
    case same_format:
      output_format_ = input_format_;
      break;
    case ascii_format:
      output_format_ = ply::ascii_format;
      break;
    case binary_format:
      output_format_ = ply::host_byte_order == ply::little_endian_byte_order ? ply::binary_little_endian_format : ply::binary_big_endian_format;
      break;
    case binary_big_endian_format:
      output_format_ = ply::binary_big_endian_format;
      break;
    case binary_little_endian_format:
      output_format_ = ply::binary_little_endian_format;
      break;
  };

  (*ostream_) << "format ";
  switch (output_format_) {
    case ply::ascii_format:
      (*ostream_) << "ascii";
      break;
    case ply::binary_little_endian_format:
      (*ostream_) << "binary_little_endian";
      break;
    case ply::binary_big_endian_format:
      (*ostream_) << "binary_big_endian";
      break;
  }
  (*ostream_) << " " << version << "\n";
}

void ply_to_ply_converter::element_begin_callback()
{
  if (output_format_ == ply::ascii_format) {
    bol_ = true;
  }
}

void ply_to_ply_converter::element_end_callback()
{
  if (output_format_ == ply::ascii_format) {
    (*ostream_) << "\n";
  }
}

std::tr1::tuple<std::tr1::function<void()>, std::tr1::function<void()> > ply_to_ply_converter::element_definition_callback(const std::string& element_name, std::size_t count)
{
  (*ostream_) << "element " << element_name << " " << count << "\n";
  return std::tr1::tuple<std::tr1::function<void()>, std::tr1::function<void()> >(
    std::tr1::bind(&ply_to_ply_converter::element_begin_callback, this),
    std::tr1::bind(&ply_to_ply_converter::element_end_callback, this)
  );
}

template <typename ScalarType>
void ply_to_ply_converter::x_property_callback(ScalarType scalar)
{
  Repository::current_vertex[0] = scalar;
  ply_to_ply_converter::scalar_property_callback(scalar);
}

template <typename ScalarType>
void ply_to_ply_converter::y_property_callback(ScalarType scalar)
{
  Repository::current_vertex[1] = scalar;
  ply_to_ply_converter::scalar_property_callback(scalar);
}

template <typename ScalarType>
void ply_to_ply_converter::z_property_callback(ScalarType scalar)
{
  Repository::current_vertex[2] = scalar;
  if (Repository::is_loading_boundary) {
    BoundaryChecker::add_vertex_from_repository();
  } else {
    float3 point;
    point.Set(Repository::current_vertex[0], Repository::current_vertex[1], Repository::current_vertex[2]);
    if (BoundaryChecker::polyhedron.Contains(point)) {
      ply_to_ply_converter::scalar_property_callback(scalar);
    } else {
      (*ostream_) << "DEL";
    }
  }
}

template <typename ScalarType>
void ply_to_ply_converter::scalar_property_callback(ScalarType scalar)
{
  if (Repository::is_loading_boundary) {
    return;
  }
  if (output_format_ == ply::ascii_format) {
    using namespace ply::io_operators;
    if (bol_) {
      bol_ = false;
      (*ostream_) << scalar;
    }
    else {
      (*ostream_) << " " << scalar;
    }
  }
  else {
    if (((ply::host_byte_order == ply::little_endian_byte_order) && (output_format_ == ply::binary_big_endian_format))
      || ((ply::host_byte_order == ply::big_endian_byte_order) && (output_format_ == ply::binary_little_endian_format))) {
      ply::swap_byte_order(scalar);
    }
    ostream_->write(reinterpret_cast<char*>(&scalar), sizeof(scalar));
  }
}

template <typename ScalarType>
std::tr1::function<void (ScalarType)> ply_to_ply_converter::scalar_property_definition_callback(const std::string& element_name, const std::string& property_name)
{
  (*ostream_) << "property " << ply::type_traits<ScalarType>::old_name() << " " << property_name << "\n";

  if (element_name == "vertex") {
    if (property_name == "x") {
      return std::tr1::bind(&ply_to_ply_converter::x_property_callback<ScalarType>, this, _1);
    } else if (property_name == "y") {
      return std::tr1::bind(&ply_to_ply_converter::y_property_callback<ScalarType>, this, _1);
    } else if (property_name == "z") {
      return std::tr1::bind(&ply_to_ply_converter::z_property_callback<ScalarType>, this, _1);
    } else {
      return std::tr1::bind(&ply_to_ply_converter::scalar_property_callback<ScalarType>, this, _1);
    }
  }
}

template <typename SizeType, typename ScalarType>
void ply_to_ply_converter::list_property_begin_callback(SizeType size)
{
  if (output_format_ == ply::ascii_format) {
    using namespace ply::io_operators;
    if (bol_) {
      bol_ = false;
      (*ostream_) << size;
    }
    else {
      (*ostream_) << " " << size;
    }
  }
  else {
    if (((ply::host_byte_order == ply::little_endian_byte_order) && (output_format_ == ply::binary_big_endian_format))
      || ((ply::host_byte_order == ply::big_endian_byte_order) && (output_format_ == ply::binary_little_endian_format))) {
      ply::swap_byte_order(size);
    }
    ostream_->write(reinterpret_cast<char*>(&size), sizeof(size));
  }
}

template <typename SizeType, typename ScalarType>
void ply_to_ply_converter::list_property_face_callback(ScalarType scalar)
{
  Repository::current_face[Repository::current_face_index] = scalar;
  Repository::current_face_index++;
  if (Repository::current_face_index > 2) {
    BoundaryChecker::add_face_from_repository();
    Repository::current_face_index = 0;
  }

  // This is copy-pasted from ply_to_ply_converter::list_property_element_callback(ScalarType scalar), because I don't know C++.
  if (output_format_ == ply::ascii_format) {
    using namespace ply::io_operators;
    (*ostream_) << " " << scalar;
  }
  else {
    if (((ply::host_byte_order == ply::little_endian_byte_order) && (output_format_ == ply::binary_big_endian_format))
      || ((ply::host_byte_order == ply::big_endian_byte_order) && (output_format_ == ply::binary_little_endian_format))) {
      ply::swap_byte_order(scalar);
    }
    ostream_->write(reinterpret_cast<char*>(&scalar), sizeof(scalar));
  }
}

template <typename SizeType, typename ScalarType>
void ply_to_ply_converter::list_property_element_callback(ScalarType scalar)
{
  if (output_format_ == ply::ascii_format) {
    using namespace ply::io_operators;
    (*ostream_) << " " << scalar;
  }
  else {
    if (((ply::host_byte_order == ply::little_endian_byte_order) && (output_format_ == ply::binary_big_endian_format))
      || ((ply::host_byte_order == ply::big_endian_byte_order) && (output_format_ == ply::binary_little_endian_format))) {
      ply::swap_byte_order(scalar);
    }
    ostream_->write(reinterpret_cast<char*>(&scalar), sizeof(scalar));
  }
}

template <typename SizeType, typename ScalarType>
void ply_to_ply_converter::list_property_end_callback()
{
}

template <typename SizeType, typename ScalarType>
std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> > ply_to_ply_converter::list_property_definition_callback(const std::string& element_name, const std::string& property_name)
{
// XXX
  (*ostream_) << "property list " << ply::type_traits<SizeType>::old_name() << " " << ply::type_traits<ScalarType>::old_name() << " " << property_name << "\n";
  if (element_name == "face") {
    return std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> >(
      std::tr1::bind(&ply_to_ply_converter::list_property_begin_callback<SizeType, ScalarType>, this, _1),
      std::tr1::bind(&ply_to_ply_converter::list_property_face_callback<SizeType, ScalarType>, this, _1),
      std::tr1::bind(&ply_to_ply_converter::list_property_end_callback<SizeType, ScalarType>, this)
    );
  } else {
    return std::tr1::tuple<std::tr1::function<void (SizeType)>, std::tr1::function<void (ScalarType)>, std::tr1::function<void ()> >(
      std::tr1::bind(&ply_to_ply_converter::list_property_begin_callback<SizeType, ScalarType>, this, _1),
      std::tr1::bind(&ply_to_ply_converter::list_property_element_callback<SizeType, ScalarType>, this, _1),
      std::tr1::bind(&ply_to_ply_converter::list_property_end_callback<SizeType, ScalarType>, this)
    );
  }
}

void ply_to_ply_converter::comment_callback(const std::string& comment)
{
  (*ostream_) << comment << "\n";
}

void ply_to_ply_converter::obj_info_callback(const std::string& obj_info)
{
  (*ostream_) << obj_info << "\n";
}

bool ply_to_ply_converter::end_header_callback()
{
  (*ostream_) << "end_header" << "\n";
  return true;
}

bool ply_to_ply_converter::load_boundary(std::istream& istream, std::ostream& ostream)
{
  ply::ply_parser::flags_type ply_parser_flags = 0;

  ply::ply_parser ply_parser(ply_parser_flags);

  std::string ifilename;

  ply_parser.info_callback(std::tr1::bind(&ply_to_ply_converter::info_callback, this, std::tr1::ref(ifilename), _1, _2));
  ply_parser.warning_callback(std::tr1::bind(&ply_to_ply_converter::warning_callback, this, std::tr1::ref(ifilename), _1, _2));
  ply_parser.error_callback(std::tr1::bind(&ply_to_ply_converter::error_callback, this, std::tr1::ref(ifilename), _1, _2));

  ply_parser.magic_callback(std::tr1::bind(&ply_to_ply_converter::magic_callback, this));
  ply_parser.format_callback(std::tr1::bind(&ply_to_ply_converter::format_callback, this, _1, _2));
  ply_parser.element_definition_callback(std::tr1::bind(&ply_to_ply_converter::element_definition_callback, this, _1, _2));

  ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;

  ply::at<ply::int8>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int8>, this, _1, _2);
  ply::at<ply::int16>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int16>, this, _1, _2);
  ply::at<ply::int32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int32>, this, _1, _2);
  ply::at<ply::uint8>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint8>, this, _1, _2);
  ply::at<ply::uint16>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint16>, this, _1, _2);
  ply::at<ply::uint32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint32>, this, _1, _2);
  ply::at<ply::float32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::float32>, this, _1, _2);
  ply::at<ply::float64>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::float64>, this, _1, _2);

  ply_parser.scalar_property_definition_callbacks(scalar_property_definition_callbacks);

  ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;

  ply::at<ply::uint8, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int8>, this, _1, _2);
  ply::at<ply::uint8, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int16>, this, _1, _2);
  ply::at<ply::uint8, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int32>, this, _1, _2);
  ply::at<ply::uint8, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint8>, this, _1, _2);
  ply::at<ply::uint8, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint16>, this, _1, _2);
  ply::at<ply::uint8, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint32>, this, _1, _2);
  ply::at<ply::uint8, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::float32>, this, _1, _2);
  ply::at<ply::uint8, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::float64>, this, _1, _2);

  ply::at<ply::uint16, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int8>, this, _1, _2);
  ply::at<ply::uint16, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int16>, this, _1, _2);
  ply::at<ply::uint16, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int32>, this, _1, _2);
  ply::at<ply::uint16, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint8>, this, _1, _2);
  ply::at<ply::uint16, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint16>, this, _1, _2);
  ply::at<ply::uint16, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint32>, this, _1, _2);
  ply::at<ply::uint16, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::float32>, this, _1, _2);
  ply::at<ply::uint16, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::float64>, this, _1, _2);

  ply::at<ply::uint32, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int8>, this, _1, _2);
  ply::at<ply::uint32, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int16>, this, _1, _2);
  ply::at<ply::uint32, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int32>, this, _1, _2);
  ply::at<ply::uint32, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint8>, this, _1, _2);
  ply::at<ply::uint32, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint16>, this, _1, _2);
  ply::at<ply::uint32, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint32>, this, _1, _2);
  ply::at<ply::uint32, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::float32>, this, _1, _2);
  ply::at<ply::uint32, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::float64>, this, _1, _2);

  ply_parser.list_property_definition_callbacks(list_property_definition_callbacks);

  ply_parser.comment_callback(std::tr1::bind(&ply_to_ply_converter::comment_callback, this, _1));
  ply_parser.obj_info_callback(std::tr1::bind(&ply_to_ply_converter::obj_info_callback, this, _1));
  ply_parser.end_header_callback(std::tr1::bind(&ply_to_ply_converter::end_header_callback, this));

  ostream_ = &ostream;
  return ply_parser.parse(istream);
}

bool ply_to_ply_converter::convert(std::istream& istream, std::ostream& ostream)
{
  ply::ply_parser::flags_type ply_parser_flags = 0;

  ply::ply_parser ply_parser(ply_parser_flags);

  std::string ifilename;

  ply_parser.info_callback(std::tr1::bind(&ply_to_ply_converter::info_callback, this, std::tr1::ref(ifilename), _1, _2));
  ply_parser.warning_callback(std::tr1::bind(&ply_to_ply_converter::warning_callback, this, std::tr1::ref(ifilename), _1, _2));
  ply_parser.error_callback(std::tr1::bind(&ply_to_ply_converter::error_callback, this, std::tr1::ref(ifilename), _1, _2));

  ply_parser.magic_callback(std::tr1::bind(&ply_to_ply_converter::magic_callback, this));
  ply_parser.format_callback(std::tr1::bind(&ply_to_ply_converter::format_callback, this, _1, _2));
  ply_parser.element_definition_callback(std::tr1::bind(&ply_to_ply_converter::element_definition_callback, this, _1, _2));

  ply::ply_parser::scalar_property_definition_callbacks_type scalar_property_definition_callbacks;

  ply::at<ply::int8>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int8>, this, _1, _2);
  ply::at<ply::int16>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int16>, this, _1, _2);
  ply::at<ply::int32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::int32>, this, _1, _2);
  ply::at<ply::uint8>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint8>, this, _1, _2);
  ply::at<ply::uint16>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint16>, this, _1, _2);
  ply::at<ply::uint32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::uint32>, this, _1, _2);
  ply::at<ply::float32>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::float32>, this, _1, _2);
  ply::at<ply::float64>(scalar_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::scalar_property_definition_callback<ply::float64>, this, _1, _2);

  ply_parser.scalar_property_definition_callbacks(scalar_property_definition_callbacks);

  ply::ply_parser::list_property_definition_callbacks_type list_property_definition_callbacks;

  ply::at<ply::uint8, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int8>, this, _1, _2);
  ply::at<ply::uint8, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int16>, this, _1, _2);
  ply::at<ply::uint8, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::int32>, this, _1, _2);
  ply::at<ply::uint8, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint8>, this, _1, _2);
  ply::at<ply::uint8, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint16>, this, _1, _2);
  ply::at<ply::uint8, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::uint32>, this, _1, _2);
  ply::at<ply::uint8, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::float32>, this, _1, _2);
  ply::at<ply::uint8, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint8, ply::float64>, this, _1, _2);

  ply::at<ply::uint16, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int8>, this, _1, _2);
  ply::at<ply::uint16, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int16>, this, _1, _2);
  ply::at<ply::uint16, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::int32>, this, _1, _2);
  ply::at<ply::uint16, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint8>, this, _1, _2);
  ply::at<ply::uint16, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint16>, this, _1, _2);
  ply::at<ply::uint16, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::uint32>, this, _1, _2);
  ply::at<ply::uint16, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::float32>, this, _1, _2);
  ply::at<ply::uint16, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint16, ply::float64>, this, _1, _2);

  ply::at<ply::uint32, ply::int8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int8>, this, _1, _2);
  ply::at<ply::uint32, ply::int16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int16>, this, _1, _2);
  ply::at<ply::uint32, ply::int32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::int32>, this, _1, _2);
  ply::at<ply::uint32, ply::uint8>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint8>, this, _1, _2);
  ply::at<ply::uint32, ply::uint16>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint16>, this, _1, _2);
  ply::at<ply::uint32, ply::uint32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::uint32>, this, _1, _2);
  ply::at<ply::uint32, ply::float32>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::float32>, this, _1, _2);
  ply::at<ply::uint32, ply::float64>(list_property_definition_callbacks) = std::tr1::bind(&ply_to_ply_converter::list_property_definition_callback<ply::uint32, ply::float64>, this, _1, _2);

  ply_parser.list_property_definition_callbacks(list_property_definition_callbacks);

  ply_parser.comment_callback(std::tr1::bind(&ply_to_ply_converter::comment_callback, this, _1));
  ply_parser.obj_info_callback(std::tr1::bind(&ply_to_ply_converter::obj_info_callback, this, _1));
  ply_parser.end_header_callback(std::tr1::bind(&ply_to_ply_converter::end_header_callback, this));

  ostream_ = &ostream;
  return ply_parser.parse(istream);
}

int main(int argc, char* argv[])
{
  ply_to_ply_converter::format_type ply_to_ply_converter_format = ply_to_ply_converter::same_format;

  int argi;
  for (argi = 1; argi < argc; ++argi) {

    if (argv[argi][0] != '-') {
      break;
    }
    if (argv[argi][1] == 0) {
      ++argi;
      break;
    }
    char short_opt, *long_opt, *opt_arg;
    if (argv[argi][1] != '-') {
      short_opt = argv[argi][1];
      opt_arg = &argv[argi][2];
      long_opt = &argv[argi][2];
      while (*long_opt != '\0') {
        ++long_opt;
      }
    }
    else {
      short_opt = 0;
      long_opt = &argv[argi][2];
      opt_arg = long_opt;
      while ((*opt_arg != '=') && (*opt_arg != '\0')) {
        ++opt_arg;
      }
      if (*opt_arg == '=') {
        *opt_arg++ = '\0';
      }
    }

    if ((short_opt == 'h') || (std::strcmp(long_opt, "help") == 0)) {
      std::cout << "Usage: point_cloud_cleaner [OPTION] <BOUNDARYFILE> [[INFILE] OUTFILE]\n";
      std::cout << "Parse a triangulated PLY file, and remove all vertices outside a bounding polyhedron.\n";
      std::cout << "\n";
      std::cout << "  -h, --help           display this help and exit\n";
      std::cout << "  -v, --version        output version information and exit\n";
      std::cout << "  -f, --format=FORMAT  set format\n";
      std::cout << "\n";
      std::cout << "FORMAT may be one of the following: ascii, binary, binary_big_endian,\n";
      std::cout << "binary_little_endian.\n";
      std::cout << "If no format is given, the format of INFILE is kept.\n";
      std::cout << "\n";
      std::cout << "With no INFILE/OUTFILE, or when INFILE/OUTFILE is -, read standard input/output.\n";
      return EXIT_SUCCESS;
    }

    else if ((short_opt == 'v') || (std::strcmp(long_opt, "version") == 0)) {
      std::cout << "point_cloud_cleaner v0.1\n";
      std::cout << "Copyright (C) 2015 Dion Moult <dion@thinkmoult.com>\n";
      std::cout << "\n";
      std::cout << "This program is free software; you can redistribute it and/or modify\n";
      std::cout << "it under the terms of the GNU General Public License as published by\n";
      std::cout << "the Free Software Foundation; either version 2 of the License, or\n";
      std::cout << "(at your option) any later version.\n";
      std::cout << "\n";
      std::cout << "This program is distributed in the hope that it will be useful,\n";
      std::cout << "but WITHOUT ANY WARRANTY; without even the implied warranty of\n";
      std::cout << "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n";
      std::cout << "GNU General Public License for more details.\n";
      std::cout << "\n";
      std::cout << "You should have received a copy of the GNU General Public License\n";
      std::cout << "along with this program; if not, write to the Free Software\n";
      std::cout << "Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA\n";
      return EXIT_SUCCESS;
    }

    else if ((short_opt == 'f') || (std::strcmp(long_opt, "format") == 0)) {
      if (strcmp(opt_arg, "ascii") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::ascii_format;
      }
      else if (strcmp(opt_arg, "binary") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_format;
      }
      else if (strcmp(opt_arg, "binary_little_endian") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_little_endian_format;
      }
      else if (strcmp(opt_arg, "binary_big_endian") == 0) {
        ply_to_ply_converter_format = ply_to_ply_converter::binary_big_endian_format;
      }
      else {
        std::cerr << "point_cloud_cleaner: " << "invalid option `" << argv[argi] << "'" << "\n";
        std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
        return EXIT_FAILURE;
      }
    }

    else {
      std::cerr << "point_cloud_cleaner: " << "invalid option `" << argv[argi] << "'" << "\n";
      std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
      return EXIT_FAILURE;
    }
  }

  int parc = argc - argi;
  char** parv = argv + argi;
  if (parc > 3) {
    std::cerr << "point_cloud_cleaner: " << "too many parameters" << "\n";
    std::cerr << "Try `" << argv[0] << " --help' for more information.\n";
    return EXIT_FAILURE;
  }

  std::ifstream bfstream;
  const char* bfilename = "";
  if (parc > 0) {
    bfilename = parv[0];
    if (std::strcmp(bfilename, "-") != 0) {
      bfstream.open(bfilename);
      if (!bfstream.is_open()) {
        std::cerr << "point_cloud_cleaner: " << bfilename << ": " << "no such file or directory" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::ifstream ifstream;
  const char* ifilename = "";
  if (parc > 1) {
    ifilename = parv[1];
    if (std::strcmp(ifilename, "-") != 0) {
      ifstream.open(ifilename);
      if (!ifstream.is_open()) {
        std::cerr << "point_cloud_cleaner: " << ifilename << ": " << "no such file or directory" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::ofstream ofstream;
  const char* ofilename = "";
  if (parc > 2) {
    ofilename = parv[2];
    if (std::strcmp(ofilename, "-") != 0) {
      ofstream.open(ofilename);
      if (!ofstream.is_open()) {
        std::cerr << "point_cloud_cleaner: " << ofilename << ": " << "could not open file" << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  std::istream& bstream = bfstream;
  std::istream& istream = ifstream.is_open() ? ifstream : std::cin;
  std::ostream& ostream = ofstream.is_open() ? ofstream : std::cout;

  class ply_to_ply_converter ply_to_ply_converter(ply_to_ply_converter_format);
  ply_to_ply_converter.load_boundary(bstream, ostream);
  Repository::is_loading_boundary = false;
  int result = ply_to_ply_converter.convert(istream, ostream);
  std::cout << "Loaded boundary polygon ...\n";
  std::cout << "Boundary vertices loaded: ";
  std::cout << BoundaryChecker::polyhedron.NumVertices();
  std::cout << "\n";
  std::cout << "Boundary faces loaded: ";
  std::cout << BoundaryChecker::polyhedron.NumFaces();
  std::cout << "\n";
  std::cout << "All done, please run the following commands to clean up:\n";
  std::cout << "sed -i '1,/ply/d' " << parv[2] << "\n";
  std::cout << "sed -i '1s/^/ply\\n/' " << parv[2] << "\n";
  std::cout << "sed -i '/DEL/d' " << parv[2] << "\n";
  std::cout << "VERTNUM=$(expr $(wc -l < " << parv[2] << ") - 13)\n";
  std::cout << "sed -i \"s/element vertex .*/element vertex $VERTNUM/\" " << parv[2] << "\n";
  return result;
}
