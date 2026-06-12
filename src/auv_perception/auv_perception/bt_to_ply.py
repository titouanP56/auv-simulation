#!/usr/bin/env python3
import os
import sys
import subprocess
import tempfile

def main():
    if len(sys.argv) == 1:
        # Default behavior: use the autosave file in the package
        import ament_index_python
        try:
            pkg_path = ament_index_python.get_package_share_directory('auv_perception')
            # go up from share/auv_perception to src
            src_path = os.path.abspath(os.path.join(pkg_path, '../../../../src/auv_perception'))
            input_file = os.path.join(src_path, 'net_map_autosave.bt')
            output_file = os.path.join(src_path, 'net_map_autosave.ply')
        except:
            input_file = 'net_map_autosave.bt'
            output_file = 'net_map_autosave.ply'
    elif len(sys.argv) == 3:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
    else:
        print("Usage: ros2 run auv_perception bt_to_ply [input.bt] [output.ply]")
        print("       (If no arguments are provided, it defaults to converting net_map_autosave.bt in the source folder)")
        sys.exit(1)

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found.")
        sys.exit(1)

    # C++ snippet to parse octomap and save as PLY
    cpp_code = """
#include <octomap/octomap.h>
#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) return 1;
    octomap::OcTree tree(argv[1]);
    std::vector<octomap::point3d> points;
    for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            points.push_back(it.getCoordinate());
        }
    }
    std::ofstream out(argv[2]);
    out << "ply\\nformat ascii 1.0\\nelement vertex " << points.size() << "\\n"
        << "property float x\\nproperty float y\\nproperty float z\\nend_header\\n";
    for(size_t i=0; i<points.size(); ++i) {
        out << points[i].x() << " " << points[i].y() << " " << points[i].z() << "\\n";
    }
    out.close();
    std::cout << "\\033[92mSuccessfully exported " << points.size() << " points to " << argv[2] << "\\033[0m" << std::endl;
    return 0;
}
"""

    print(f"Converting '{input_file}' -> '{output_file}'...")

    with tempfile.TemporaryDirectory() as tmpdir:
        cpp_file = os.path.join(tmpdir, "bt_to_ply.cpp")
        exe_file = os.path.join(tmpdir, "bt_to_ply")

        with open(cpp_file, 'w') as f:
            f.write(cpp_code)

        # Compile the C++ snippet
        compile_cmd = ["g++", cpp_file, "-o", exe_file, "-loctomap", "-loctomath"]
        res = subprocess.run(compile_cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print(f"Compilation error! Make sure liboctomap-dev is installed.\n{res.stderr}")
            sys.exit(1)

        # Run the executable
        run_cmd = [exe_file, input_file, output_file]
        res = subprocess.run(run_cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print(f"Execution error!\n{res.stderr}")
            sys.exit(1)
        
        print(res.stdout.strip())

if __name__ == '__main__':
    main()
