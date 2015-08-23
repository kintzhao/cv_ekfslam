/**
 * Copyright (C) 2010  ARToolkitPlus Authors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *  Daniel Wagner
 */

#ifndef __ARTOOLKITAR_HEADERFILE__
#define __ARTOOLKITAR_HEADERFILE__

#include <stdlib.h>

#include <ARToolKitPlus/config.h>
#include <stdint.h>

#define arMalloc(V,T,S)  \
{ if( ((V) = (T *)malloc( sizeof(T) * (S) )) == 0 ) \
{printf("malloc error!!\n"); exit(1);} }

namespace ARToolKitPlus
{
    /*ARMarkerInfo
    int 	area
    double 	cf
    int 	dir
    int 	id
    double 	line [4][3]
    double 	pos [2]
    double 	vertex [4][2]
    Detailed Description

    main structure for detected marker.

    Store information after contour detection (in idea screen coordinate, after distorsion compensated).

    Remarks:
        lines are represented by 3 values a,b,c for ax+by+c=0

    Parameters:
        area	number of pixels in the labeled region
        id	marker identitied number
        dir	Direction that tells about the rotation about the marker (possible values are 0, 1, 2 or 3). This parameter makes it possible to tell about the line order of the detected marker (so which line is the first one) and so find the first vertex. This is important to compute the transformation matrix in arGetTransMat().
        cf	confidence value (probability to be a marker)
        pos	center of marker (in ideal screen coordinates)
        line	line equations for four side of the marker (in ideal screen coordinates)
        vertex	edge points of the marker (in ideal screen coordinates) */
    typedef struct {
        int area;
        int id;
        int dir;
        ARFloat cf;
        ARFloat pos[2];
        ARFloat line[4][3];
        ARFloat vertex[4][2];
    } AR_EXPORT ARMarkerInfo;
    /*ARMarkerInfo2
        Public Attributes
        int 	area
        int 	coord_num
        double 	pos [2]
        int 	vertex [5]
        int 	x_coord [AR_CHAIN_MAX]
        int 	y_coord [AR_CHAIN_MAX]
        Detailed Description

        internal structure use for marker detection.

        Store information after contour detection (in observed screen coordinate, before distorsion correction).

        Parameters:
            area	number of pixels in the labeled region
            pos	position of the center of the marker (in observed screen coordinates)
            coord_num	numer of pixels in the contour.
            x_coord	x coordinate of the pixels of contours (size limited by AR_CHAIN_MAX).
            y_coord	y coordinate of the pixels of contours (size limited by AR_CHAIN_MAX).
            vertex	position of the vertices of the marker. (in observed screen coordinates) rem:the first vertex is stored again as the 5th entry in the array � for convenience of drawing a line-strip easier.
*/
    typedef struct {
        int area;
        ARFloat pos[2];
        int coord_num;
        int x_coord[AR_CHAIN_MAX];
        int y_coord[AR_CHAIN_MAX];
        int vertex[5];
    } AR_EXPORT ARMarkerInfo2;

    typedef struct {
        ARMarkerInfo marker;
        int count;
    } AR_EXPORT arPrevInfo;

} // namespace ARToolKitPlus


#endif  //__ARTOOLKITAR_HEADERFILE__
