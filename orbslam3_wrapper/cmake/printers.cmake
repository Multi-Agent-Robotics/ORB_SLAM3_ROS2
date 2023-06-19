string(ASCII 27 ESC)
set(RESET "${ESC}[0m")
set(RED "${ESC}[0;31m")
set(GREEN "${ESC}[0;32m")
SET(YELLOW "${ESC}[0;33m")
set(BLUE "${ESC}[0;34m")
set(MAGENTA "${ESC}[0;35m")
set(CYAN "${ESC}[0;36m")
set(WHITE "${ESC}[0;37m")
set(BOLDBLACK "${ESC}[1m${ESC}[30m")
set(BOLDRED "${ESC}[1m${ESC}[31m")
set(BOLDGREEN "${ESC}[1m${ESC}[32m")
set(BOLDYELLOW "${ESC}[1m${ESC}[33m")
set(BOLDBLUE "${ESC}[1m${ESC}[34m")
set(BOLDMAGENTA "${ESC}[1m${ESC}[35m")
set(BOLDCYAN "${ESC}[1m${ESC}[36m")
set(BOLDWHITE "${ESC}[1m${ESC}[37m")
set(BOLD "${ESC}[1m")
set(DIM "${ESC}[2m")
set(ITALIC "${ESC}[3m")
set(UNDERLINE "${ESC}[4m")
set(BLINK "${ESC}[5m")
set(REVERSE "${ESC}[7m")
set(HIDDEN "${ESC}[8m")
set(CHECKMARK "✔")
set(CHECKMARK_COLOR ${GREEN})
set(CROSS "✘")
set(CROSS_COLOR ${RED})

# NOTE: this has to be a macro because it needs to modify a global variable
set(__PRINTVAR_LENGTH_OF_LONGEST_VAR_NAME 0)

macro(printvar var)
  string(LENGTH ${var} var_length)

  if(${var_length} GREATER ${__PRINTVAR_LENGTH_OF_LONGEST_VAR_NAME})
    set(__PRINTVAR_LENGTH_OF_LONGEST_VAR_NAME ${var_length})
  endif()

  set(padding "")
  math(EXPR padding_length "${__PRINTVAR_LENGTH_OF_LONGEST_VAR_NAME} - ${var_length}")

  foreach(i RANGE ${padding_length})
    set(padding "${padding} ")
  endforeach()

  # if var is a list of variables, print each one recursively
  # if var is empty or undefined, print in red
  if(NOT DEFINED ${var} OR "${${var}}" STREQUAL "")
    message(STATUS "${RED}${var}${RESET}${padding} = <empty>")
  else()
    message(STATUS "${GREEN}${var}${RESET}${padding} = ${${var}}")
  endif()
endmacro()

# Takes a list and prints it with each element enumerated and right-aligned
# e.g.:
# 0. foo
# 1. bar
# 2. baz
function(print_list_enumerated var)
  # copy the list so we can sort it
  # set(copy ${${var}})
  # list(SORT ${copy})
  list(LENGTH ${var} COUNT)

  if(${COUNT} GREATER 9)
    set(width 2)
  elseif(${COUNT} GREATER 99)
    set(width 3)
  elseif(${COUNT} GREATER 999)
    set(width 4)
  else()
    set(width 1)
  endif()

  set(i 0)

  # alternate between bold and normal text
  foreach(elem ${${var}})
    string(LENGTH ${i} i_length)
    set(padding "")
    math(EXPR padding_length "${width} - ${i_length}")

    foreach(j RANGE ${padding_length})
      set(padding "${padding} ")
    endforeach()

    # right-align the numbers
    math(EXPR remainder "${i} % 2")
    set(markup ${RESET})

    if(${remainder} EQUAL 0)
      set(markup ${BOLD})
    endif()

    # check if elem is a generator expression .i.e. starts with $< and ends with >
    # if so, print it in blue
    string(REGEX MATCH "^\\$<.*>$" is_generator_expression ${elem})

    if(is_generator_expression)
      set(markup "${BLUE}")
    endif()

    message("${padding}${i}. ${markup}${elem}${RESET}")
    math(EXPR i "${i} + 1")
  endforeach()
endfunction()

function(printlist var)
  list(LENGTH ${var} COUNT)
  message(STATUS "${GREEN}${var}${RESET} = (${CYAN}${COUNT}${RESET} elements)")
  set(i 0)

  foreach(f ${${var}})
    message(STATUS " ${i}. ${f}")
    math(EXPR i "${i} + 1")
  endforeach()
endfunction()

function(tree dir)
  message(STATUS "${GREEN}${dir}${RESET}")
  file(GLOB children RELATIVE ${dir} ${dir}/*)

  foreach(child ${children})
    if(IS_DIRECTORY ${dir}/${child})
      tree(${dir}/${child})
    else()
      message(STATUS "  ${dir}/${child}")
    endif()
  endforeach()
endfunction()

function(print_target_properties target)
  # get all the properties of the target
  get_target_property(TARGET_PROPERTIES ${target} PROPERTIES)

  # get the length of the longest property name
  set(__PRINTVAR_LENGTH_OF_LONGEST_PROPERTY_NAME 0)

  foreach(property ${TARGET_PROPERTIES})
    string(LENGTH ${property} property_length)

    if(${property_length} GREATER ${__PRINTVAR_LENGTH_OF_LONGEST_PROPERTY_NAME})
      set(__PRINTVAR_LENGTH_OF_LONGEST_PROPERTY_NAME ${property_length})
    endif()
  endforeach()

  # print the target name
  message(STATUS "${GREEN}${target}${RESET} properties:")

  # print each property
  foreach(property ${TARGET_PROPERTIES})
    string(LENGTH ${property} property_length)
    set(padding "")
    math(EXPR padding_length "${__PRINTVAR_LENGTH_OF_LONGEST_PROPERTY_NAME} - ${property_length}")

    foreach(i RANGE ${padding_length})
      set(padding "${padding} ")
    endforeach()

    message(STATUS "${GREEN}${property}${RESET}${padding} = ${${property}}")
  endforeach()
endfunction()

function(print_target_linked_libraries target)
  get_target_property(LINKED_LIBRARIES ${target} LINK_LIBRARIES)
  list(LENGTH LINKED_LIBRARIES LINKED_LIBRARIES_COUNT)
  message(STATUS "target ${GREEN}${target}${RESET} links against ${CYAN}${LINKED_LIBRARIES_COUNT}${RESET} libraries:")

  foreach(link_lib ${LINKED_LIBRARIES})
    # if the library is a shared object file, check if it exists
    set(color ${BOLD})

    if(${link_lib} MATCHES "\\.so")
      if(NOT EXISTS ${link_lib})
        set(color ${RED})
        message(" - [${CROSS_COLOR}${CROSS}${RESET}] ${color}${link_lib}${RESET}")
        continue()
      else()
        message(" - [${CHECKMARK_COLOR}${CHECKMARK}${RESET}] ${color}${link_lib}${RESET}")
        continue()
      endif()
    endif()

    message(" - ${color}${link_lib}${RESET}")
  endforeach()
endfunction()

function(print_target_included_directories target)
  get_target_property(INCLUDE_DIRECTORIES ${target} INCLUDE_DIRECTORIES)
  list(LENGTH INCLUDE_DIRECTORIES INCLUDE_DIRECTORIES_COUNT)
  message(STATUS "target ${GREEN}${target}${RESET} include directories ${CYAN}${INCLUDE_DIRECTORIES_COUNT}${RESET}: (${CHECKMARK_COLOR}${CHECKMARK}${RESET} = directory exists, ${CROSS_COLOR}${CROSS}${RESET} = does not exist, ${BLUE}$${RESET} = generator expression)")
  set(directories_exist_count 0)
  set(number_of_directories_that_are_generator_expressions 0)

  foreach(include_dir ${INCLUDE_DIRECTORIES})
    # check if the include directory is a generator expression
    string(REGEX MATCH "^\\$<.*>$" is_generator_expression ${include_dir})

    if(is_generator_expression)
      math(EXPR number_of_directories_that_are_generator_expressions "${number_of_directories_that_are_generator_expressions} + 1")
      set(mark "$")
      set(color ${BLUE})
    elseif(NOT EXISTS ${include_dir})
      # check if the include directory exists
      set(mark ${CROSS})
      set(color ${CROSS_COLOR})
    else()
      set(mark ${CHECKMARK})
      set(color ${CHECKMARK_COLOR})
      math(EXPR directories_exist_count "${directories_exist_count} + 1")
    endif()

    message(" - [${color}${mark}${RESET}] ${BOLD}${include_dir}${RESET}")
  endforeach()

  message(STATUS "${number_of_directories_that_are_generator_expressions} of ${INCLUDE_DIRECTORIES_COUNT} directories are generator expressions")
  math(EXPR directories_exist_count "${directories_exist_count} + ${number_of_directories_that_are_generator_expressions}")

  if(${number_of_directories_that_are_generator_expressions} EQUAL ${INCLUDE_DIRECTORIES_COUNT})
    message(STATUS "all directories are generator expressions")
  else()
    message(STATUS "${directories_exist_count} of ${INCLUDE_DIRECTORIES_COUNT} directories exist")
  endif()
endfunction()

function(print_target_sources target)
  get_target_property(SOURCES ${target} SOURCES)
  list(LENGTH SOURCES SOURCES_COUNT)

  message(STATUS "target ${GREEN}${target}${RESET} sources ${CYAN}${SOURCES_COUNT}${RESET}: (${CHECKMARK_COLOR}${CHECKMARK}${RESET} = exists, ${CROSS_COLOR}${CROSS}${RESET} = does not exist)")
  set(sources_exist_count 0)

  foreach(source ${SOURCES})
    # if the source file does not exist, print in bold
    if(NOT EXISTS ${source})
      set(color ${RED})
      set(mark ${CROSS})
      set(mark_color ${CROSS_COLOR})
    else()
      set(color ${BOLD})
      set(mark ${CHECKMARK})
      set(mark_color ${CHECKMARK_COLOR})
      math(EXPR sources_exist_count "${sources_exist_count} + 1")
    endif()

    message(" - [${mark_color}${mark}${RESET}] ${color}${source}${RESET}")
  endforeach()

  message(STATUS "${sources_exist_count} of ${SOURCES_COUNT} sources exist")
endfunction()

function(print_target_compile_definitions target)
  get_target_property(COMPILE_DEFINITIONS ${target} COMPILE_DEFINITIONS)

  if(${COMPILE_DEFINITIONS} STREQUAL "COMPILE_DEFINITIONS-NOTFOUND")
    message(STATUS "compile definitions not found for target ${GREEN}${target}${RESET}")
    return()
  endif()

  list(LENGTH COMPILE_DEFINITIONS COMPILE_DEFINITIONS_COUNT)
  message(STATUS "target ${GREEN}${target}${RESET} compile definitions ${CYAN}${COMPILE_DEFINITIONS_COUNT}${RESET}:")

  foreach(compile_definition ${COMPILE_DEFINITIONS})
    message(STATUS " - ${compile_definition}")
  endforeach()

  # message(STATUS "")
endfunction()

function(print_property_of_target target property)
  get_target_property(${property} ${target} ${property})
  list(LENGTH ${property} property_count)

  if(${property_count} EQUAL 1)
    if(${${property}} STREQUAL "${property}-NOTFOUND")
      message(STATUS "${property} not found for target ${GREEN}${target}${RESET}")
      return()
    endif()
  endif()

  message(STATUS "target ${GREEN}${target}${RESET} ${property} (${CYAN}${property_count}${RESET}):")
  print_list_enumerated(${property})
endfunction()

function(print_target_compile_features target)
  print_property_of_target(${target} COMPILE_FEATURES)
endfunction()

function(print_target_compile_options target)
  print_property_of_target(${target} COMPILE_OPTIONS)
endfunction()

function(print_target_compile_definitions target)
  print_property_of_target(${target} COMPILE_DEFINITIONS)
endfunction()

function(print_target_link_options target)
  print_property_of_target(${target} LINK_OPTIONS)
endfunction()

# function(print_target_compile_options target)
# get_target_property(COMPILE_OPTIONS ${target} COMPILE_OPTIONS)
# if (${COMPILE_OPTIONS} STREQUAL "COMPILE_OPTIONS-NOTFOUND")
# message(STATUS "compile options not found for target ${GREEN}${target}${RESET}")
# return()
# endif()
# list(LENGTH COMPILE_OPTIONS COMPILE_OPTIONS_COUNT)
# message(STATUS "target ${GREEN}${target}${RESET} compile options ${CYAN}${COMPILE_OPTIONS_COUNT}${RESET}:")
# foreach(compile_option ${COMPILE_OPTIONS})
# message(STATUS " - ${compile_option}")
# endforeach()
# #   message(STATUS "")
# endfunction()

# target_link_directories()
function(print_target_link_directories target)
  get_target_property(LINK_DIRECTORIES ${target} LINK_DIRECTORIES)

  if(${LINK_DIRECTORIES} STREQUAL "LINK_DIRECTORIES-NOTFOUND")
    message(STATUS "link directories not found for target ${GREEN}${target}${RESET}")
    return()
  endif()

  list(LENGTH LINK_DIRECTORIES LINK_DIRECTORIES_COUNT)
  message(STATUS "target ${GREEN}${target}${RESET} link directories ${CYAN}${LINK_DIRECTORIES_COUNT}${RESET}:")

  foreach(link_directory ${LINK_DIRECTORIES})
    message(STATUS " - ${link_directory}")
  endforeach()

  # message(STATUS "")
endfunction()

function(get_terminal_columns output_var)
  execute_process(
    COMMAND bash "-c" "tput cols"
    OUTPUT_VARIABLE terminal_columns
    RESULT_VARIABLE result
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(NOT result EQUAL 0)
    set(terminal_columns 80)
  endif()

  set(${output_var} ${terminal_columns} PARENT_SCOPE)
endfunction()

function(hr char n)
  set(line "")

  foreach(i RANGE ${n})
    set(line "${line}${char}")
  endforeach()

  message("${line}")
endfunction()

function(print_target_information target)
  # determine if target is an executable or a library
  get_target_property(target_type ${target} TYPE)

  if(${target_type} STREQUAL "EXECUTABLE")
    set(target_type "executable")
  elseif(${target_type} STREQUAL "STATIC_LIBRARY")
    set(target_type "static library")
  elseif(${target_type} STREQUAL "SHARED_LIBRARY")
    set(target_type "shared library")
  else()
    set(target_type "unknown")
  endif()

  if(${target_type} STREQUAL "unknown")
    message(WARNING "target ${GREEN}${target}${RESET} is an unknown type")
    return()
  endif()

  # set(n 100)
  get_terminal_columns(NUM_TERMINAL_COLUMNS)

  # subtract 4 for the delimiters and 2 for the spaces
  math(EXPR n "${NUM_TERMINAL_COLUMNS} - 4")

  # set(n ${NUM_TERMINAL_COLUMNS})
  set(delim "-")
  hr("=" ${n})
  message(STATUS "target ${GREEN}${target}${RESET} is a ${CYAN}${target_type}${RESET}")
  hr(${delim} ${n})
  print_target_properties(${target})
  hr(${delim} ${n})
  print_target_included_directories(${target})
  hr(${delim} ${n})
  print_target_linked_libraries(${target})
  hr(${delim} ${n})
  print_target_link_directories(${target})
  hr(${delim} ${n})
  print_target_sources(${target})
  hr(${delim} ${n})
  print_target_compile_definitions(${target})
  hr(${delim} ${n})
  print_target_compile_features(${target})
  hr(${delim} ${n})
  print_target_compile_options(${target})
  hr(${delim} ${n})
  print_target_link_options(${target})
  hr("=" ${n})
endfunction()

function(print_what_find_package_found pkg)
  # print what find_package found
  message(STATUS "find_package(${pkg}) found:")
  message(STATUS " - ${CYAN}${${pkg}_FOUND}${RESET}")
  message(STATUS " - ${CYAN}${${pkg}_VERSION}${RESET}")
  message(STATUS " - ${CYAN}${${pkg}_INCLUDE_DIRS}${RESET}")

  foreach(lib ${${pkg}_LIBRARIES})
    message(STATUS " - ${CYAN}${lib}${RESET}")
  endforeach()

  message(STATUS "")

  foreach(lib ${${pkg}_LIBRARIES})
    # print_target_information(${lib})
    message(STATUS "target ${GREEN}${lib}${RESET} is a ${CYAN}library${RESET}")
  endforeach()

  message(STATUS "")
endfunction()

function(print_cmake_host_system_information)
  message(STATUS "cmake host system information:")
  set(keys NUMBER_OF_LOGICAL_CORES NUMBER_OF_PHYSICAL_CORES HOSTNAME FQDN TOTAL_VIRTUAL_MEMORY AVAILABLE_VIRTUAL_MEMORY TOTAL_PHYSICAL_MEMORY AVAILABLE_PHYSICAL_MEMORY IS_64BIT)

  # if cmake --version >= 3.10 add the following keys
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.10")
    set(keys_available_with_cmake_3_10 NUMBER_OF_LOGICAL_CORES NUMBER_OF_PHYSICAL_CORES HOSTNAME FQDN TOTAL_VIRTUAL_MEMORY AVAILABLE_VIRTUAL_MEMORY TOTAL_PHYSICAL_MEMORY AVAILABLE_PHYSICAL_MEMORY IS_64BIT PROCESSOR_ARCHITECTURE PROCESSOR_IDENTIFIER PROCESSOR_LEVEL PROCESSOR_REVISION)

    foreach(key ${keys_available_with_cmake_3_10})
      list(APPEND keys ${key})
    endforeach()
  endif()

  set(length_of_longest_key 0)

  foreach(key ${keys})
    string(LENGTH ${key} key_length)

    if(${key_length} GREATER ${length_of_longest_key})
      set(length_of_longest_key ${key_length})
    endif()
  endforeach()

  foreach(key ${keys})
    cmake_host_system_information(RESULT value QUERY ${key})

    if(${value} EQUAL 1)
      set(value "${GREEN}yes${RESET}")
    elseif(${value} EQUAL "0")
      set(value "${RED}no${RESET}")
    endif()

    message(STATUS "- ${MAGENTA}${key}${RESET}: ${value}")
  endforeach()

  # Queries system information of the host system on which cmake runs. One or more <key> can be provided to select the information to be queried. The list of queried values is stored in <variable>.

  # <key> can be one of the following values:

  # NUMBER_OF_LOGICAL_CORES

  # Number of logical cores
  # NUMBER_OF_PHYSICAL_CORES

  # Number of physical cores
  # HOSTNAME

  # Hostname
  # FQDN

  # Fully qualified domain name
  # TOTAL_VIRTUAL_MEMORY

  # Total virtual memory in MiB [1]
  # AVAILABLE_VIRTUAL_MEMORY

  # Available virtual memory in MiB [1]
  # TOTAL_PHYSICAL_MEMORY

  # Total physical memory in MiB [1]
  # AVAILABLE_PHYSICAL_MEMORY

  # Available physical memory in MiB [1]
  # IS_64BIT

  # New in version 3.10.

  # One if processor is 64Bit
  # HAS_FPU

  # New in version 3.10.

  # One if processor has floating point unit
  # HAS_MMX

  # New in version 3.10.

  # One if processor supports MMX instructions
  # HAS_MMX_PLUS

  # New in version 3.10.

  # One if processor supports Ext. MMX instructions
  # HAS_SSE

  # New in version 3.10.

  # One if processor supports SSE instructions
  # HAS_SSE2

  # New in version 3.10.

  # One if processor supports SSE2 instructions
  # HAS_SSE_FP

  # New in version 3.10.

  # One if processor supports SSE FP instructions
  # HAS_SSE_MMX

  # New in version 3.10.

  # One if processor supports SSE MMX instructions
  # HAS_AMD_3DNOW

  # New in version 3.10.

  # One if processor supports 3DNow instructions
  # HAS_AMD_3DNOW_PLUS

  # New in version 3.10.

  # One if processor supports 3DNow+ instructions
  # HAS_IA64

  # New in version 3.10.

  # One if IA64 processor emulating x86
  # HAS_SERIAL_NUMBER

  # New in version 3.10.

  # One if processor has serial number
  # PROCESSOR_SERIAL_NUMBER

  # New in version 3.10.

  # Processor serial number
  # PROCESSOR_NAME

  # New in version 3.10.

  # Human readable processor name
  # PROCESSOR_DESCRIPTION

  # New in version 3.10.

  # Human readable full processor description
  # OS_NAME

  # New in version 3.10.

  # See CMAKE_HOST_SYSTEM_NAME
  # OS_RELEASE

  # New in version 3.10.

  # The OS sub-type e.g. on Windows Professional
  # OS_VERSION

  # New in version 3.10.

  # The OS build ID
  # OS_PLATFORM

  # New in version 3.10.
endfunction()

function(print_all_variables)
  message(STATUS "print_all_variables:")
  get_cmake_property(variable_names VARIABLES)

  foreach(variable_name ${variable_names})
    message(STATUS "${variable_name}=${${variable_name}}")
  endforeach()
endfunction()

macro(find_longest_string_in_list list_to_search output_variable_name)
  set(longest_string "")
  set(length_of_longest_string 0)

  foreach(string ${list_to_search})
    string(LENGTH ${string} string_length)

    if(${string_length} GREATER ${length_of_longest_string})
      set(length_of_longest_string ${string_length})
      set(longest_string ${string})
    endif()
  endforeach()

  set(${output_variable_name} ${longest_string} PARENT_SCOPE)
endmacro()

function(print_all_variables_with_prefix prefix)
  get_terminal_columns(NUM_TERMINAL_COLUMNS)
  math(EXPR n "${NUM_TERMINAL_COLUMNS} - 1")
  hr("=" ${n})

  set(variables_with_prefix "")

  get_cmake_property(variable_names VARIABLES)

  foreach(variable_name ${variable_names})
    string(REGEX MATCH "^${prefix}" match_result ${variable_name})

    if(match_result)
      list(APPEND variables_with_prefix ${variable_name})
    endif()
  endforeach()

  find_longest_string_in_list(${variables_with_prefix} longest_string)
  string(LENGTH ${longest_string} length_of_longest_variable_name)

  # sort the list
  list(SORT variables_with_prefix)
  list(LENGTH variables_with_prefix num_variables_with_prefix)
  set(number_of_empty_variables 0)
  message(STATUS "ALL VARIABLES WITH PREFIX: ${YELLOW}${prefix}${RESET} (${num_variables_with_prefix} variables)")

  foreach(variable_name ${variables_with_prefix})
    set(padding "")
    string(LENGTH ${variable_name} variable_name_length)
    math(EXPR padding_length "${length_of_longest_variable_name} - ${variable_name_length}")

    # string(REPEAT " " ${padding_length} padding)

    # if the variable is empty, then print it in red
    set(color ${GREEN})

    if("${${variable_name}}" STREQUAL "")
      math(EXPR number_of_empty_variables "${number_of_empty_variables} + 1")
      set(color ${RED})
    endif()

    message(" - ${color}${variable_name}${RESET}${padding} = ${${variable_name}}")
  endforeach()

  if(${number_of_empty_variables} GREATER 0)
    message("") # newline
    message(STATUS "${RED}${number_of_empty_variables} variables are empty${RESET} out of ${num_variables_with_prefix} variables with prefix ${YELLOW}${prefix}${RESET}")
  endif()

  hr("=" ${n})
endfunction()

function(print_all_variables_with_prefixes prefixes)
  foreach(prefix ${prefixes})
    print_all_variables_with_prefix(${prefix})
  endforeach()
endfunction()

function(print_all_variables_with_postfix postfix)
  get_terminal_columns(NUM_TERMINAL_COLUMNS)
  math(EXPR n "${NUM_TERMINAL_COLUMNS} - 1")
  hr("=" ${n})

  set(variables_with_postfix "")

  get_cmake_property(variable_names VARIABLES)

  foreach(variable_name ${variable_names})
    string(REGEX MATCH "${postfix}$" match_result ${variable_name})

    if(match_result)
      list(APPEND variables_with_postfix ${variable_name})
    endif()
  endforeach()

  find_longest_string_in_list(${variables_with_postfix} longest_string)
  string(LENGTH ${longest_string} length_of_longest_variable_name)

  # sort the list
  list(SORT variables_with_postfix)
  list(LENGTH variables_with_postfix num_variables_with_postfix)
  set(number_of_empty_variables 0)
  message(STATUS "ALL VARIABLES WITH POSTFIX: ${YELLOW}${postfix}${RESET} (${num_variables_with_postfix} variables)")

  foreach(variable_name ${variables_with_postfix})
    set(padding "")
    string(LENGTH ${variable_name} variable_name_length)
    math(EXPR padding_length "${length_of_longest_variable_name} - ${variable_name_length}")

    # string(REPEAT " " ${padding_length} padding)

    # if the variable is empty, then print it in red
    set(color ${GREEN})

    if("${${variable_name}}" STREQUAL "")
      math(EXPR number_of_empty_variables "${number_of_empty_variables} + 1")
      set(color ${RED})
    endif()

    message(" - ${color}${variable_name}${RESET}${padding} = ${${variable_name}}")
  endforeach()
endfunction()

# function(get_all_targets output_variable)
# get_all_targets_with_prefix("" ${output_variable})
# endfunction()
function(print_CMAKE_variables)
  print_all_variables_with_prefix("CMAKE_")
endfunction()
