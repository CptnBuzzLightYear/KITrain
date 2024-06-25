# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_msgs:msg/GroundTruth.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GroundTruth(type):
    """Metaclass of message 'GroundTruth'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('lidar_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lidar_msgs.msg.GroundTruth')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ground_truth
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ground_truth
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ground_truth
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ground_truth
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ground_truth

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GroundTruth(metaclass=Metaclass_GroundTruth):
    """Message class 'GroundTruth'."""

    __slots__ = [
        '_id',
        '_centerx',
        '_centery',
        '_centerz',
        '_length_x',
        '_width_y',
        '_height_z',
        '_yaw',
        '_tag',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'centerx': 'float',
        'centery': 'float',
        'centerz': 'float',
        'length_x': 'float',
        'width_y': 'float',
        'height_z': 'float',
        'yaw': 'float',
        'tag': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.centerx = kwargs.get('centerx', float())
        self.centery = kwargs.get('centery', float())
        self.centerz = kwargs.get('centerz', float())
        self.length_x = kwargs.get('length_x', float())
        self.width_y = kwargs.get('width_y', float())
        self.height_z = kwargs.get('height_z', float())
        self.yaw = kwargs.get('yaw', float())
        self.tag = kwargs.get('tag', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.centerx != other.centerx:
            return False
        if self.centery != other.centery:
            return False
        if self.centerz != other.centerz:
            return False
        if self.length_x != other.length_x:
            return False
        if self.width_y != other.width_y:
            return False
        if self.height_z != other.height_z:
            return False
        if self.yaw != other.yaw:
            return False
        if self.tag != other.tag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def centerx(self):
        """Message field 'centerx'."""
        return self._centerx

    @centerx.setter
    def centerx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'centerx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'centerx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._centerx = value

    @builtins.property
    def centery(self):
        """Message field 'centery'."""
        return self._centery

    @centery.setter
    def centery(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'centery' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'centery' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._centery = value

    @builtins.property
    def centerz(self):
        """Message field 'centerz'."""
        return self._centerz

    @centerz.setter
    def centerz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'centerz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'centerz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._centerz = value

    @builtins.property
    def length_x(self):
        """Message field 'length_x'."""
        return self._length_x

    @length_x.setter
    def length_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'length_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'length_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._length_x = value

    @builtins.property
    def width_y(self):
        """Message field 'width_y'."""
        return self._width_y

    @width_y.setter
    def width_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'width_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'width_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._width_y = value

    @builtins.property
    def height_z(self):
        """Message field 'height_z'."""
        return self._height_z

    @height_z.setter
    def height_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'height_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._height_z = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value

    @builtins.property
    def tag(self):
        """Message field 'tag'."""
        return self._tag

    @tag.setter
    def tag(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tag' field must be of type 'str'"
        self._tag = value
