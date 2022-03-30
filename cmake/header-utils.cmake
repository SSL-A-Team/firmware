
# from https://stackoverflow.com/questions/17653738/recursive-cmake-search-for-header-and-source-files

MACRO(HEADER_DIRECTORIES return_list base_folder)
    FILE(GLOB_RECURSE new_list ${base_folder}/*.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()
