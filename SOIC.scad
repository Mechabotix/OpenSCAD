SOIC//SMD SOIC IC
//Based on JEDEC MS-012 Issue F Variation AB

module RenderSOIC(pins)
{
    if((pins < 8) || (pins >16))
    {
        pins = 14; //Set pins to 14 if out of range
    }
    
    D = (((pins/2)-1)*1.25008)+1.14976; //Length of body - depends on number of pins - estimated from Spec


    E1 = 3.9; //Body width
    A1 = 0.175; //body clearance above floor. 0.1 Min. 0.15 Max.
    A = 1.55; //Height of top of body above floor. 1.35 Min.  1.75 Max.
    E = 6; //Width of body + legs
    e = 1.27; //BSC distance between adjacent legs
    b = 0.41; //width of leg leads
    c = 0.175; //height of leg leads
    R = 0.1; //Radius of leg turn > 0.07
    L1 = 1.04; //The total horizontal length of the leg
    Theta = 5; //degrees of theta 1 - the angle the legs rise off the ground
    Theta1 = 4; //4 x Theta1 is degrees the side edges of the body tilt in. Maximim 8 deg.
    Theta2 = 5; //degrees of theta 2 - the angle of the middle section from vertical

    //Other Dimensions
    LegSection1 = 0.2; //The first section from the side of the body
    LegSection2 = 0.2; //The second straight section

    // **** Calculations for Leg Module ****
    //Calculate how far to displace the centre of Leg Section 2 from origin
    LegSection2Hyp = sqrt(((LegSection2/2)*(LegSection2/2))+((c/2)*(c/2)));
    ThetaSection2 = atan(LegSection2/c);
    ThetaSection2Center = ThetaSection2 - Theta2;
    LegSection2CenterX = LegSection1 + (R*cos(Theta2)) + (LegSection2Hyp * cos(ThetaSection2Center));
    LegSection2CenterZ = (c/2) + R - (R*sin(Theta2)) + (LegSection2Hyp * sin(ThetaSection2Center));

    //Leg Bend 2 Calculations
    ThetaBend2 = 90-Theta2-Theta;
    LegBend2XTrans = LegSection1 + (R*cos(Theta2)) + 2*(LegSection2Hyp * cos(ThetaSection2Center))+(R*cos(Theta2));
    LegBend2YTrans = (c/2) + R - (R*sin(Theta2)) + 2*(LegSection2Hyp * sin(ThetaSection2Center))-(R*sin(Theta2));

    //Leg Section 3 Calculations (Needs Fixing)
    XOriginToLeg3Left = (LegSection1 + (R*cos(Theta2)) + 2*(LegSection2Hyp * cos(ThetaSection2Center))+(R*cos(Theta2)))-((R+c)*sin(Theta));
    LegSection3Horizintal = L1 - (c*sin(Theta)) - (XOriginToLeg3Left); //which equals LegSection3*Cos(Theta)
    LegSection3 = LegSection3Horizintal / cos(Theta);

    ThetaP = atan((R+(c/2))/(LegSection3/2));
    ThetaQ = 90 - ThetaP - Theta;
    LegSection3deltaHyp = sqrt(((R+(c/2))*(R+(c/2)))+((LegSection3/2)*(LegSection3/2)));
    LegSection3deltaX = LegSection3deltaHyp * sin(ThetaQ); //X distance from centre of bend2 to centre of LegSection3
    LegSection3deltaY = LegSection3deltaHyp * cos(ThetaQ); //Y distance from centre of bend2 to centre of LegSection3
    LegSection3XTrans = LegBend2XTrans + LegSection3deltaX;
    LegSection3ZTrans = LegBend2YTrans + LegSection3deltaY;

    //calculate distace from origin to bottom of leg section 3 so the pin can be shifted up the z-axis
    LegZTranslate = LegSection3ZTrans + (c/2)*cos(Theta) + (LegSection3/2)*sin(Theta);

    module AddLeg(x,y,rotZ)
    {
        //This module creates a leg/pin.
        


        //IC Leg
        color("LightGrey")
        translate([x,y,LegZTranslate])//translate leg based on values passed to module
        rotate([0,0,rotZ])//rotate the leg around Z based on values passed to module
        union()
        {
            //Leg Section 1
            translate([LegSection1/2,0,0])
                cube([LegSection1,b,c], center = true);

            //Leg Bend 1 - 
            translate([LegSection1,0,-(R+(c/2))])
                rotate([90,0,0])
                    difference()
                    {
                        difference()
                        {
                           difference()
                            {
                                difference()
                                {
                                    //Radius corner for leg
                                    cylinder(r=(R+c), h=b, $fa=1, $fs=0.05, center = true);
                                    cylinder(r=R, h=(b+0.1), $fa=1, $fs=0.05, center = true);
                                }
                                //Clear off the left side of the cylinder to make a 90 deg angle
                                translate ([-(((2*R)+c)/2),0,0]) //shift -x direction by 1/2 the width of the block
                                    cube([((2*R)+c),((4*R)+c),b+0.1], center = true);
                            }
                            //Create a block angled up 5 degrees
                            rotate([0,0,Theta2])
                                translate ([((2*R)+c)/2,-((2*R)+c)/2,0])
                                    cube([((2*R)+c),((2*R)+c),b+0.2], center = true);
                        }
                        //Clear out any remaining parts of the cylinder in the bottom half
                        translate([0,-(((2*R)+c)/2),0])//shift -y direction by 1/2 the height of the block
                            rotate([0,0,0])
                                cube([((4*R)+c),((2*R)+c),b+0.3], center= true);   
                    };
                    
            //Leg Section 2
            translate([LegSection2CenterX,0,-LegSection2CenterZ])
                  rotate([0,-Theta2,0])
                    cube([c,b,LegSection2], center = true);
                    
            //Leg Bend 2
            translate([LegBend2XTrans,0,-LegBend2YTrans])
                rotate([90,0,0])
                    difference()
                    {
                        difference()
                        {
                            //Radius corner for leg
                            cylinder(r=(R+c), h=b, $fa=1, $fs=0.05, center = true);
                            cylinder(r=R, h=(b+0.1), $fa=1, $fs=0.05, center = true);
                        }
                        union()
                        {
                            translate([0,(R+c+0.05)/2,0])
                                cube([2*(R+c+0.05),(R+c+0.05),b+0.15], center = true);

                            translate([((R+c+0.05)/2),-((R+c+0.05)/2),0])
                                cube([(R+c+0.05),(R+c+0.055),b+0.15], center = true);

                            translate([0,0,-(b+0.15)/2])
                                rotate([0,0,90+Theta])
                                    cube([(R+c+0.05),(R+c+0.05),b+0.15], center = false);

                            translate([0,0,-(b+0.15)/2])
                                rotate([0,0,-(90+Theta2)])
                                    cube([(R+c+0.05),(R+c+0.05),b+0.15], center = false);
                        }
                    };

            //Leg Section 3
            translate([LegSection3XTrans,0,-LegSection3ZTrans])
                rotate([0,Theta,0])
                    cube([LegSection3,b,c], center = true);
        }
    }



    //Add Legs using module
    for (p = [1:(pins/2)])
    {
        AddLeg(E1/2,(e*p)-(((pins/2)*e/2)+e/2),0);
        AddLeg(-E1/2,(e*p)-(((pins/2)*e/2)+e/2),180);
    }

    //Polyhedron Body

    //Inset Calculations for top and bottom of body
    InsetT = tan(4*Theta1) * (A-(LegZTranslate + (c/2)));
    InsetB = tan(4*Theta1) * (LegZTranslate - ((c/2)+A1));

    color("DarkSlateGrey")
    polyhedron(
    points=
    [
        [E1/2-InsetT,D/2-InsetT,A],[E1/2-InsetT,-(D/2-InsetT),A],[-(E1/2-InsetT),-(D/2-InsetT),A],[-(E1/2-InsetT),(D/2-InsetT),A], //top
        [E1/2,D/2,LegZTranslate+(c/2)],[E1/2,-D/2,LegZTranslate+(c/2)],[-E1/2,-D/2,LegZTranslate+(c/2)],[-E1/2,D/2,LegZTranslate+(c/2)],
        [E1/2,D/2,LegZTranslate-(c/2)],[E1/2,-D/2,LegZTranslate-(c/2)],[-E1/2,-D/2,LegZTranslate-(c/2)],[-E1/2,D/2,LegZTranslate-(c/2)],
        [E1/2-InsetB,D/2-InsetB,A1],[E1/2-InsetB,-(D/2-InsetB),A1],[-(E1/2-InsetB),-(D/2-InsetB),A1],[-(E1/2-InsetB),D/2-InsetB,A1] //bottom
    ],
    faces =
    [
        [0,1,3],[1,2,3], //Top
        [0,4,5],[0,5,1],[1,5,6],[1,6,2],[2,6,3],[3,6,7],[3,4,0],[3,7,4],
        [4,8,9],[4,9,5],[5,9,10],[5,10,6],[6,10,11],[6,11,7],[7,8,4],[7,11,8], //sides where pins exit
        [8,12,13],[8,13,9],[9,13,14],[9,14,10],[10,14,15],[10,15,11],[11,15,12],[11,12,8],
        [12,15,13],[13,15,14] //Bottom
    ]
    );
}
RenderSOIC(8);
