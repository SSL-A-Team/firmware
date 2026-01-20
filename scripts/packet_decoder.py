"""
C Header Parser and Packet Decoder for ateam-common-packets

This module provides functionality to parse C header files containing struct
and enum definitions, and decode binary packets according to those definitions.
"""

import struct
import re
import os

# C type to Python struct format and size
C_TYPE_MAP = {
    'uint8_t': ('B', 1),
    'int8_t': ('b', 1),
    'uint16_t': ('H', 2),
    'int16_t': ('h', 2),
    'uint32_t': ('I', 4),
    'int32_t': ('i', 4),
    'uint64_t': ('Q', 8),
    'int64_t': ('q', 8),
    'float': ('f', 4),
    'double': ('d', 8),
    'char': ('c', 1),
    'unsigned char': ('B', 1),
}

class StructField:
    """Represents a field in a C struct"""
    def __init__(self, name, c_type, array_size=None, bitfield_width=None):
        self.name = name
        self.c_type = c_type
        self.array_size = array_size  # None if not an array
        self.bitfield_width = bitfield_width  # None if not a bitfield

    def __repr__(self):
        arr = f"[{self.array_size}]" if self.array_size else ""
        bf = f":{self.bitfield_width}" if self.bitfield_width else ""
        return f"{self.c_type} {self.name}{arr}{bf}"

class StructDef:
    """Represents a C struct definition"""
    def __init__(self, name):
        self.name = name
        self.fields = []
        self.size = None  # From assert_size if available

    def add_field(self, field):
        self.fields.append(field)

    def __repr__(self):
        fields_str = "\n  ".join(str(f) for f in self.fields)
        return f"struct {self.name} ({self.size} bytes) {{\n  {fields_str}\n}}"

class EnumDef:
    """Represents a C enum definition"""
    def __init__(self, name):
        self.name = name
        self.values = {}  # value -> name mapping

    def add_value(self, name, value):
        self.values[value] = name

    def get_name(self, value):
        return self.values.get(value, f"UNKNOWN({value})")

class HeaderParser:
    """Parses C header files to extract struct and enum definitions"""

    def __init__(self):
        self.structs = {}
        self.enums = {}
        self.typedefs = {}  # typedef name -> actual type

    def parse_file(self, filepath):
        """Parse a single header file"""
        with open(filepath, 'r') as f:
            content = f.read()

        # Remove comments
        content = re.sub(r'//.*$', '', content, flags=re.MULTILINE)
        content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)

        # Parse enums
        self._parse_enums(content)

        # Parse structs
        self._parse_structs(content)

        # Parse assert_size statements
        self._parse_assert_sizes(content)

    def _parse_enums(self, content):
        """Parse enum definitions"""
        # Match: typedef enum EnumName { ... } EnumName;
        enum_pattern = r'typedef\s+enum\s+(\w+)\s*\{([^}]+)\}'
        for match in re.finditer(enum_pattern, content):
            enum_name = match.group(1)
            enum_body = match.group(2)

            enum_def = EnumDef(enum_name)
            current_value = 0

            for line in enum_body.split(','):
                line = line.strip()
                if not line:
                    continue

                # Match: NAME = value or just NAME
                value_match = re.match(r'(\w+)\s*=\s*(\d+)', line)
                if value_match:
                    name = value_match.group(1)
                    current_value = int(value_match.group(2))
                else:
                    name_match = re.match(r'(\w+)', line)
                    if name_match:
                        name = name_match.group(1)
                    else:
                        continue

                enum_def.add_value(name, current_value)
                current_value += 1

            self.enums[enum_name] = enum_def

    def _parse_structs(self, content):
        """Parse struct definitions"""
        # Match: typedef struct StructName { ... } StructName;
        struct_pattern = r'typedef\s+struct\s+(\w+)\s*\{([^}]+)\}\s*(?:__attribute__\s*\(\([^)]*\)\)\s*)?(\w+)\s*;'
        for match in re.finditer(struct_pattern, content):
            struct_name = match.group(1)
            struct_body = match.group(2)
            typedef_name = match.group(3)

            struct_def = StructDef(struct_name)

            # Parse each field
            for line in struct_body.split(';'):
                line = line.strip()
                if not line:
                    continue

                field = self._parse_field(line)
                if field:
                    struct_def.add_field(field)

            self.structs[struct_name] = struct_def
            if typedef_name != struct_name:
                self.typedefs[typedef_name] = struct_name

    def _parse_field(self, line):
        """Parse a single struct field"""
        line = line.strip()
        if not line:
            return None

        # Check for bitfield: type name : width
        bitfield_match = re.match(r'(\w+)\s+(\w+)\s*:\s*(\d+)', line)
        if bitfield_match:
            c_type = bitfield_match.group(1)
            name = bitfield_match.group(2)
            width = int(bitfield_match.group(3))
            return StructField(name, c_type, bitfield_width=width)

        # Check for array: type name[size]
        array_match = re.match(r'(\w+)\s+(\w+)\s*\[(\d+)\]', line)
        if array_match:
            c_type = array_match.group(1)
            name = array_match.group(2)
            size = int(array_match.group(3))
            return StructField(name, c_type, array_size=size)

        # Check for simple field: type name
        simple_match = re.match(r'(\w+)\s+(\w+)', line)
        if simple_match:
            c_type = simple_match.group(1)
            name = simple_match.group(2)
            return StructField(name, c_type)

        return None

    def _parse_assert_sizes(self, content):
        """Parse assert_size statements to get struct sizes"""
        size_pattern = r'assert_size\s*\(\s*(\w+)\s*,\s*(\d+)\s*\)'
        for match in re.finditer(size_pattern, content):
            struct_name = match.group(1)
            size = int(match.group(2))
            if struct_name in self.structs:
                self.structs[struct_name].size = size

    def get_struct(self, name):
        """Get a struct by name, following typedefs"""
        if name in self.structs:
            return self.structs[name]
        if name in self.typedefs:
            return self.structs.get(self.typedefs[name])
        return None

    def get_enum(self, name):
        """Get an enum by name"""
        return self.enums.get(name)

class PacketDecoder:
    """Decodes binary packets using parsed struct definitions"""

    def __init__(self, parser):
        self.parser = parser

    def decode(self, struct_name, data, offset=0):
        """Decode data according to a struct definition"""
        struct_def = self.parser.get_struct(struct_name)
        if not struct_def:
            print(f"Unknown struct: {struct_name}")
            return None

        result = {}
        current_offset = offset
        bitfield_accumulator = 0
        bitfield_bits_used = 0
        bitfield_type = None

        for field in struct_def.fields:
            if field.bitfield_width:
                # Handle bitfield
                if bitfield_type is None or bitfield_type != field.c_type:
                    # Start new bitfield group
                    type_info = C_TYPE_MAP.get(field.c_type)
                    if type_info:
                        fmt, size = type_info
                        if current_offset + size <= len(data):
                            bitfield_accumulator = struct.unpack('<' + fmt, data[current_offset:current_offset+size])[0]
                            current_offset += size
                    bitfield_bits_used = 0
                    bitfield_type = field.c_type

                # Extract bits
                mask = (1 << field.bitfield_width) - 1
                value = (bitfield_accumulator >> bitfield_bits_used) & mask
                bitfield_bits_used += field.bitfield_width

                result[field.name] = value
            else:
                # Reset bitfield state
                bitfield_type = None
                bitfield_bits_used = 0

                # Check if this is a nested struct
                nested_struct = self.parser.get_struct(field.c_type)
                if nested_struct:
                    if field.array_size:
                        # Array of structs
                        arr_result = []
                        for i in range(field.array_size):
                            nested_result = self.decode(field.c_type, data, current_offset)
                            if nested_result:
                                arr_result.append(nested_result)
                                current_offset += nested_struct.size or 0
                        result[field.name] = arr_result
                    else:
                        # Single nested struct
                        nested_result = self.decode(field.c_type, data, current_offset)
                        result[field.name] = nested_result
                        current_offset += nested_struct.size or 0
                else:
                    # Primitive type
                    type_info = C_TYPE_MAP.get(field.c_type)
                    if type_info:
                        fmt, size = type_info
                        if field.array_size:
                            # Array of primitives
                            arr_result = []
                            for i in range(field.array_size):
                                if current_offset + size <= len(data):
                                    value = struct.unpack('<' + fmt, data[current_offset:current_offset+size])[0]
                                    arr_result.append(value)
                                    current_offset += size
                            result[field.name] = arr_result
                        else:
                            # Single primitive
                            if current_offset + size <= len(data):
                                value = struct.unpack('<' + fmt, data[current_offset:current_offset+size])[0]
                                result[field.name] = value
                                current_offset += size
                    else:
                        # Check if it's an enum type
                        enum_def = self.parser.get_enum(field.c_type)
                        if enum_def:
                            # Treat enum as uint8
                            if current_offset + 1 <= len(data):
                                value = data[current_offset]
                                result[field.name] = value
                                result[f"{field.name}_name"] = enum_def.get_name(value)
                                current_offset += 1

        return result

    def print_decoded(self, struct_name, decoded, indent=0):
        """Pretty print decoded struct"""
        prefix = "  " * indent
        struct_def = self.parser.get_struct(struct_name)

        if not decoded:
            print(f"{prefix}(null)")
            return

        for field in struct_def.fields:
            if field.name not in decoded:
                continue

            value = decoded[field.name]

            # Check if it's a nested struct
            nested_struct = self.parser.get_struct(field.c_type)
            if nested_struct:
                if field.array_size:
                    print(f"{prefix}{field.name}: [")
                    for i, item in enumerate(value):
                        print(f"{prefix}  [{i}]:")
                        self.print_decoded(field.c_type, item, indent + 2)
                    print(f"{prefix}]")
                else:
                    print(f"{prefix}{field.name}:")
                    self.print_decoded(field.c_type, value, indent + 1)
            elif isinstance(value, list):
                # Array of primitives - show summary
                if len(value) > 5:
                    avg = sum(value) / len(value) if value else 0
                    print(f"{prefix}{field.name}: [{value[0]}, {value[1]}, ... {value[-1]}] (avg: {avg:.1f})")
                else:
                    print(f"{prefix}{field.name}: {value}")
            elif field.c_type == 'float':
                print(f"{prefix}{field.name}: {value:.4f}")
            elif f"{field.name}_name" in decoded:
                # Enum with name
                print(f"{prefix}{field.name}: {decoded[f'{field.name}_name']} ({value})")
            elif field.bitfield_width:
                # Bitfield - show as bool if 1 bit
                if field.bitfield_width == 1:
                    print(f"{prefix}{field.name}: {'ERROR' if value else 'OK'}")
                else:
                    print(f"{prefix}{field.name}: {value}")
            else:
                print(f"{prefix}{field.name}: {value}")
