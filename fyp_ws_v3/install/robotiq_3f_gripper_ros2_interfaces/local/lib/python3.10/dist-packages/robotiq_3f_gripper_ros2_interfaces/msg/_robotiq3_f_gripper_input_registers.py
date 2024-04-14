# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robotiq_3f_gripper_ros2_interfaces:msg/Robotiq3FGripperInputRegisters.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Robotiq3FGripperInputRegisters(type):
    """Metaclass of message 'Robotiq3FGripperInputRegisters'."""

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
            module = import_type_support('robotiq_3f_gripper_ros2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robotiq_3f_gripper_ros2_interfaces.msg.Robotiq3FGripperInputRegisters')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robotiq3_f_gripper_input_registers
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robotiq3_f_gripper_input_registers
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robotiq3_f_gripper_input_registers
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robotiq3_f_gripper_input_registers
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robotiq3_f_gripper_input_registers

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Robotiq3FGripperInputRegisters(metaclass=Metaclass_Robotiq3FGripperInputRegisters):
    """Message class 'Robotiq3FGripperInputRegisters'."""

    __slots__ = [
        '_g_act',
        '_g_mod',
        '_g_gto',
        '_g_imc',
        '_g_sta',
        '_g_flt',
        '_g_pra',
    ]

    _fields_and_field_types = {
        'g_act': 'uint8',
        'g_mod': 'uint8',
        'g_gto': 'uint8',
        'g_imc': 'uint8',
        'g_sta': 'uint8',
        'g_flt': 'uint8',
        'g_pra': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.g_act = kwargs.get('g_act', int())
        self.g_mod = kwargs.get('g_mod', int())
        self.g_gto = kwargs.get('g_gto', int())
        self.g_imc = kwargs.get('g_imc', int())
        self.g_sta = kwargs.get('g_sta', int())
        self.g_flt = kwargs.get('g_flt', int())
        self.g_pra = kwargs.get('g_pra', int())

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
        if self.g_act != other.g_act:
            return False
        if self.g_mod != other.g_mod:
            return False
        if self.g_gto != other.g_gto:
            return False
        if self.g_imc != other.g_imc:
            return False
        if self.g_sta != other.g_sta:
            return False
        if self.g_flt != other.g_flt:
            return False
        if self.g_pra != other.g_pra:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def g_act(self):
        """Message field 'g_act'."""
        return self._g_act

    @g_act.setter
    def g_act(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_act' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_act' field must be an unsigned integer in [0, 255]"
        self._g_act = value

    @builtins.property
    def g_mod(self):
        """Message field 'g_mod'."""
        return self._g_mod

    @g_mod.setter
    def g_mod(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_mod' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_mod' field must be an unsigned integer in [0, 255]"
        self._g_mod = value

    @builtins.property
    def g_gto(self):
        """Message field 'g_gto'."""
        return self._g_gto

    @g_gto.setter
    def g_gto(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_gto' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_gto' field must be an unsigned integer in [0, 255]"
        self._g_gto = value

    @builtins.property
    def g_imc(self):
        """Message field 'g_imc'."""
        return self._g_imc

    @g_imc.setter
    def g_imc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_imc' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_imc' field must be an unsigned integer in [0, 255]"
        self._g_imc = value

    @builtins.property
    def g_sta(self):
        """Message field 'g_sta'."""
        return self._g_sta

    @g_sta.setter
    def g_sta(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_sta' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_sta' field must be an unsigned integer in [0, 255]"
        self._g_sta = value

    @builtins.property
    def g_flt(self):
        """Message field 'g_flt'."""
        return self._g_flt

    @g_flt.setter
    def g_flt(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_flt' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_flt' field must be an unsigned integer in [0, 255]"
        self._g_flt = value

    @builtins.property
    def g_pra(self):
        """Message field 'g_pra'."""
        return self._g_pra

    @g_pra.setter
    def g_pra(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_pra' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'g_pra' field must be an unsigned integer in [0, 255]"
        self._g_pra = value
