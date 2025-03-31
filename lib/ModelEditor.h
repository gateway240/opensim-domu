
#ifndef MODEL_EDITOR_H
#define MODEL_EDITOR_H

#include <filesystem>
#include <regex>

std::string delete_imus(const std::filesystem::path &osim_file, const std::vector<std::string> &keys,
const std::string& output_suffix);

void parse_osim_file(const std::string &osim_file);

void find_and_replace(const std::ifstream &file, const std::string &start_key,
                      const std::string &end_key, const std::regex &pattern,
                      const std::string &new_substring);
void find_and_delete(std::vector<std::string> &lines, const std::string &start_key,
                     const std::string &end_key);

#endif // MODEL_EDITOR_H