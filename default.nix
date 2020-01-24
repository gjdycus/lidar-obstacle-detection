with import <nixpkgs> {};
stdenv.mkDerivation rec
{
  name = "lidar-obstacle-detection";
  env = buildEnv { name = name; paths = buildInputs; };

  LD_LIBRARY_PATH = stdenv.lib.makeLibraryPath buildInputs;

  buildInputs = [
    boost
    cmake
    pcl
  ];
}
