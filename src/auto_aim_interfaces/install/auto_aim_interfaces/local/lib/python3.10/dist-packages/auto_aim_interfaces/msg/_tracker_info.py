# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/TrackerInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrackerInfo(type):
    """Metaclass of message 'TrackerInfo'."""

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
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.TrackerInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tracker_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tracker_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tracker_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tracker_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tracker_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrackerInfo(metaclass=Metaclass_TrackerInfo):
    """Message class 'TrackerInfo'."""

    __slots__ = [
        '_is_tracking',
        '_tracking_score',
    ]

    _fields_and_field_types = {
        'is_tracking': 'boolean',
        'tracking_score': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_tracking = kwargs.get('is_tracking', bool())
        self.tracking_score = kwargs.get('tracking_score', float())

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
        if self.is_tracking != other.is_tracking:
            return False
        if self.tracking_score != other.tracking_score:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_tracking(self):
        """Message field 'is_tracking'."""
        return self._is_tracking

    @is_tracking.setter
    def is_tracking(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_tracking' field must be of type 'bool'"
        self._is_tracking = value

    @builtins.property
    def tracking_score(self):
        """Message field 'tracking_score'."""
        return self._tracking_score

    @tracking_score.setter
    def tracking_score(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tracking_score' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tracking_score' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tracking_score = value
