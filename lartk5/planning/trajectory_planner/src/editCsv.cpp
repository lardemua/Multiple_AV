#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>

int main()
{
  vector<double> matrix;
  //readfile
  fstream file;
  file.open("read.csv");
  string line
  while (getline( file, line,'\n'))  
	{
	  istringstream templine(line);
	  string data;
	  while (getline( templine, data,',')) 
	  {
	    matrix.push_back(atof(data.c_str())); 
	  }
	}
  file.close();
  
  //writefile
  file.open("write.csv");
  for (int i=0;i<matrix.size();i++)
  {
    file << matrix[i]<<",";
  }
  file.close();
  return 0;
}