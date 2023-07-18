/*
This is based on the implementation of TextArchive from the limbo library:
https://github.com/resibots/limbo/blob/master/src/limbo/serialize/text_archive.hpp
*/

#ifndef AEGEAN_TOOLS_ARCHIVE_HPP
#define AEGEAN_TOOLS_ARCHIVE_HPP

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>
#include <ctime>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

namespace aegean {
    namespace tools {
        class Archive {
        public:
            Archive(bool create_dir = true)
                : _create_dir(create_dir),
                  _fmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "")
            {
                namespace ip = boost::asio::ip;
                std::string hn = ip::host_name();

                std::time_t rawtime;
                std::tm* timeinfo;
                char buffer[80];
                std::time(&rawtime);
                timeinfo = std::localtime(&rawtime);
                std::strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);

                if (_create_dir) {
                    _dir_name = hn + "_" + std::string(buffer);
                    _create_directory();
                }
                else
                    _dir_name = ".";
            }

            const std::string& dir_name() const { return _dir_name; }

            template <typename EigenMat>
            void save(const EigenMat& v, const std::string& filename) const
            {
                _create_directory();
                std::ofstream ofs(_fname(filename));
                ofs << v.format(_fmt) << std::endl;
            }

            template <typename M>
            void load(M& m, const std::string& filename, int skip = 0,
                const char& delim = ' ') const
            {
                auto values = load(filename, skip, delim);
                m.resize(values.size(), values[0].size());
                for (size_t i = 0; i < values.size(); ++i)
                    for (size_t j = 0; j < values[i].size(); ++j)
                        m(i, j) = values[i][j];
            }

            std::vector<std::vector<double>> load(const std::string& filename, int skip = 0,
                const char& delim = ' ') const
            {
                std::ifstream ifs(filename.c_str());
                assert(ifs.good() && "Invalid file path");

                std::string line;
                std::vector<std::vector<double>> v;
                while (std::getline(ifs, line)) {
                    if (skip > 0) {
                        --skip;
                        continue;
                    }

                    std::stringstream line_stream(line);
                    std::string cell;
                    std::vector<double> line;
                    while (std::getline(line_stream, cell, delim)) {
                        (cell == "NAN") ? line.push_back(std::nan(cell.c_str()))
                                        : line.push_back(std::stod(cell));
                    }
                    v.push_back(line);
                }
                assert(!v.empty() && "Empty file");
                return v;
            }

        protected:
            void _create_directory() const
            {
                if (_create_dir) {
                    boost::filesystem::path my_path(_dir_name);
                    boost::filesystem::create_directories(my_path);
                }
            }

            std::string _fname(const std::string& f) const
            {
                return _dir_name + "/" + f + ".txt";
            }

            const bool _create_dir;
            Eigen::IOFormat _fmt;
            std::string _dir_name;
        };

    } // namespace tools
} // namespace aegean

#endif