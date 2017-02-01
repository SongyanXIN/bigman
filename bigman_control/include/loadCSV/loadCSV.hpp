/*
 * All rights reserved.
 * Copyright (C) 2016 Songyan XIN (xinsongyan@gmail.com)
 * Based on the code by Przemyslaw Kryczka, Paul O'Dowd
 *
 */
#ifndef    LOADCSV_H
#define    LOADCSV_H

#define NEWLINE  10
#define COMMA    44
#define SPACE    32
#define TAB_H    9

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>	
#include <string>
#include <iostream>	
#include <eigen3/Eigen/Dense>

//////////////////////////////////////////////////////////////////////////////////////////
// This function load csv data and return as eigen MatrixXd data type.
// Example usage:
// Eigen::MatrixXd CoM = loadCSV ("/home/user/Tests/testCSV/CoM" ); Note: no need of .csv
// to load the file "/home/user/Tests/testCSV/CoM.csv"
// In ROS, you could get the path of a file in the way:
// #include <ros/package.h>
// std::string pathToTime = ros::package::getPath("bigman_control") + "/trajectory/time";
// Eigen::MatrixXd time = loadCSV(pathToTime);
Eigen::MatrixXd loadCSV(std::string filename);

//////////////////////////////////////////////////////////////////////////////////////////
// example usage: Eigen::MatrixXd CoM; loadCSV (CoM, "/home/user/Tests/testCSV/CoM" );
// to load the file "/home/user/Tests/testCSV/CoM.csv"
bool loadCSV(Eigen::MatrixXd &data, std::string filename);

//////////////////////////////////////////////////////////////////////////////////////////
// example usage: Eigen::MatrixXd CoM; loadCSV (&CoM, "/home/user/Tests/testCSV/CoM" );
// to load the file "/home/user/Tests/testCSV/CoM.csv"
bool loadCSV(Eigen::MatrixXd *data, std::string filename);

//////////////////////////////////////////////////////////////////////////////////////////
bool loadCSV_rows(std::vector<Eigen::VectorXd> *data, char *base_file);

//////////////////////////////////////////////////////////////////////////////////////////
bool loadCSV(std::vector<std::vector<double> > *data, std::string filename);





//////////////////////////////////////////////////////////////////////////////////////////
// function definition
bool loadCSV(std::vector<std::vector<double> > *data, std::string filename) {
    using namespace std;
    using namespace Eigen;

    int file;
    int col;
    int row;
    int i, j;
    char buf[40];
    int c;
    FILE *fp;

    // printf("\n Loading the '%s'.csv filename \n", filename);

    // We try to open file zero.

    filename += ".csv";
    fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf(" - Could not open file 0! Assuming no files. Abort\n");
        return false;
    }

    // We open the file and use it to judge how many rows and 
    // columns we will need.
    fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf(" I can't open the first file \"%s\", abort\n", buf);
        return false;
    }


    // printf("\n Attempting to find out row and column dimensions...\n");
    row = 0;
    col = 0;
    i = 0;
    j = 0;
    while ((c = fgetc(fp)) != EOF) {
        if (c != COMMA && c != TAB_H && c != NEWLINE) {
            j++;
        } else if (c == COMMA || c == TAB_H) {
            i++;
            j = 0;
        } else if (c == NEWLINE) {
            if (j != 0) i++;
            if (i > col) col = i;

            i = 0;
            row++;
        }
    }
    fclose(fp);

    // printf(" I found a max of %d columns, %d rows\n", col, row);

    // Create an vectors large enough to hold all of our data    
    vector<double> tmp;
    tmp.resize(col, 0);
    data->resize(row, tmp);
    // for (int i = 0; i < col; i++)
    // data -> at(i).reserve(col);

    // Open the file to read, use sprintf to format
    // the filename string.
    fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf(" Error, could not open %s, abort\n", buf);
        return false;
    }

    // Reset variables that index the arrays
    i = 0;
    col = 0;
    row = 0;
    c = 0;
    memset(buf, '\0', sizeof(buf));

    // Read in the file data.
    while ((c = fgetc(fp)) != EOF) {

        if (c != COMMA && c != TAB_H && c != NEWLINE) {

            // We have a character
            buf[i] = c;
            i++;

        } else if (c == COMMA || c == TAB_H) {
            // We save the data, jump a column
            data->at(row).at(col) = atof(buf);
            // cout << atof( buf ) << " " ;

            // Clear the buffer
            memset(buf, '\0', sizeof(buf));

            col++;
            i = 0;

        } else if (c == NEWLINE) {
            if (i != 0) {
                // We save the data, jump a column
                data->at(row).at(col) = atof(buf);
                memset(buf, '\0', sizeof(buf));
                // cout << atof( buf ) << endl ;
            }
            // cout << atof( buf ) << endl ;
            // We want to drop a line.
            col = 0;
            i = 0;
            row++;

        }
    }

    // Close that file, ready for the next
    fclose(fp);
    // printf(" - Done\n");


    return true;

}


bool loadCSV(Eigen::MatrixXd *data, std::string filename) {
    using namespace std;
    using namespace Eigen;

    int file;
    int col;
    int row;
    int i, j;
    char buf[140];
    int c;
    FILE *fp;

    // printf("\n Loading the '%s'.csv filename \n", filename);

    // sprintf(buf, "%s.csv", filename);
    filename += ".csv";
    fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf(" - Could not open file 0! Assuming no files. Abort\n");
        return false;
    }

    // We open the file and use it to judge how many rows and 
    // columns we will need.

    // fp = fopen( filename.c_str(), "r" );
    // if( fp == NULL ) {
    // printf(" I can't open the first file \"%s\", abort\n", filename.c_str());
    // return false;
    // }    


    // printf("\n Attempting to find out row and column dimensions...\n");
    row = 0;
    col = 0;
    i = 0;
    j = 0;
    while ((c = fgetc(fp)) != EOF) {
        if (c != COMMA && c != TAB_H && c != NEWLINE) {
            j++;
        } else if (c == COMMA || c == TAB_H) {
            i++;
            j = 0;
        } else if (c == NEWLINE) {
            if (j != 0) i++;
            if (i > col) col = i;

            i = 0;
            row++;
        }
    }
    fclose(fp);

//     printf(" I found a max of %d columns, %d rows\n", col, row);

    // Create an vectors large enough to hold all of our data    
    //vector<double> tmp;
    //tmp.resize(col,0);
    data->resize(row, col);
    (*data) = Eigen::MatrixXd::Zero(row, col);
    // for (int i = 0; i < col; i++)
    // data -> at(i).reserve(col);

    // Open the file to read, use sprintf to format
    // the filename string.
    // sprintf(buf, "%s.csv", filename );

    // printf(" Opening \"%s\" to read data: ", buf);

    fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf(" Error, could not open %s, abort\n", filename.c_str());
        return false;
    }

    // Reset variables that index the arrays
    i = 0;
    col = 0;
    row = 0;
    c = 0;
    memset(buf, '\0', sizeof(buf));

    // Read in the file data.
    while ((c = fgetc(fp)) != EOF) {

        if (c != COMMA && c != TAB_H && c != NEWLINE) {

            // We have a character
            buf[i] = c;
            i++;

        } else if (c == COMMA || c == TAB_H) {
            // We save the data, jump a column
            (*data)(row, col) = atof(buf);
            // cout << atof( buf ) << " " ;

            // Clear the buffer
            memset(buf, '\0', sizeof(buf));

            col++;
            i = 0;

        } else if (c == NEWLINE) {
            if (i != 0) {
                // We save the data, jump a column
                (*data)(row, col) = atof(buf);
                memset(buf, '\0', sizeof(buf));
                // cout << atof( buf ) << endl ;
            }
            // cout << atof( buf ) << endl ;
            // We want to drop a line.
            col = 0;
            i = 0;
            row++;

        }
    }

    // Close that file, ready for the next
    fclose(fp);
    // printf(" - Done\n");


    return true;

}


bool loadCSV_rows(std::vector<Eigen::VectorXd> *data, char *filename) {

    using namespace std;
    using namespace Eigen;

    MatrixXd tmpMat;

    loadCSV(&tmpMat, filename);

    data->resize(tmpMat.rows());
    for (int i = 0; i < tmpMat.rows(); i++) {
        data->at(i) = tmpMat.block(i, 0, 1, tmpMat.cols());
    }
    return true;
}


bool loadCSV(Eigen::MatrixXd &data, std::string filename) {

    return loadCSV(&data, filename);

}


Eigen::MatrixXd loadCSV(std::string filename) {
    Eigen::MatrixXd temp;
    loadCSV(&temp, filename);
    return temp;
}


#endif
