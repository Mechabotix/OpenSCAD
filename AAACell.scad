// AAA cell
// https://github.com/Mechabotix/OpenSCAD

module RenderCell()
{
//Top of cell is origin
totalHeight = 44.5; //Height of cell from spec (Max: 44.5 Min 43.3)

bodyDia = 10.4; //Diameter from spec (Max: 10.5 not including label)
bodyChamferRad = 0.5;
baseDia = 6;
baseFilletRad = 0.3;

nippleHeight = 0.8; //Height from spec (Min: 0.8)
nippleDia = 3; //Diameter from spec (Max: 3.8)
nippleFilletRad = 0.2;
nippleShoulderDia = 6;

labelThickness = 0.005; //Thickness of plastic

//Top face of nipple
color("LightGrey")
translate([0,0,-nippleFilletRad])
    cylinder(
        h = nippleFilletRad, 
        d = (nippleDia - (2 * nippleFilletRad)),
        $fn = 100);
        
//First shoulder
color("LightGrey")
translate([0,0,-nippleFilletRad])
    rotate_extrude(convexity = 10, $fn = 100)
    translate([(nippleDia/2)-nippleFilletRad, 0, 0])
        circle(r = nippleFilletRad, $fn = 100);

//Main nipple cylinder
color("LightGrey")
translate([0,0,-(nippleHeight - nippleFilletRad)])
    cylinder(
        h = (nippleHeight - (nippleFilletRad + nippleFilletRad)),
        d = nippleDia,
        $fn = 100);

//Main nipple cylinder; base fillet
color("LightGrey")
difference()
{
    translate([0,0,nippleFilletRad - nippleHeight])
        rotate_extrude(convexity = 10, $fn = 100)
        translate([(nippleDia/2), 0, 0])
            square((nippleFilletRad),(nippleFilletRad));

    translate([0,0,(2 * nippleFilletRad) - nippleHeight])
        rotate_extrude(convexity = 10, $fn = 100)
        translate([(nippleDia/2)+ nippleFilletRad, 0, 0])
            circle(r = nippleFilletRad, $fn = 100);
}
        
//Nipple shoulder cylinder
color("LightGrey")
translate([0,0,-nippleHeight])
    cylinder(
        h = nippleFilletRad,
        d = nippleShoulderDia - (2 * nippleFilletRad),
        $fn = 100);
        
//Nipple base shoulder
color("LightGrey")
translate([0,0,-nippleHeight])
    rotate_extrude(convexity = 10, $fn = 100)
    translate([(nippleShoulderDia/2)-nippleFilletRad, 0, 0])
        circle(r = nippleFilletRad, $fn = 100);

//Top face of main body
translate([0,0,-(nippleHeight + bodyChamferRad)])
    cylinder(
        h = bodyChamferRad, 
        d = (bodyDia - (2 * bodyChamferRad)),
        $fn = 100);
        
//Body top shoulder
translate([0,0,-(nippleHeight + bodyChamferRad)])
    rotate_extrude(convexity = 10, $fn = 100)
    translate([(bodyDia/2)-bodyChamferRad, 0, 0])
        circle(r = bodyChamferRad, $fn = 100);

//Main body cylinder
translate([0,0,-(totalHeight - bodyChamferRad)])
    cylinder(
        h = (totalHeight - (nippleHeight + (2* bodyChamferRad))),
        d = bodyDia,
        $fn = 100);
        
//Body bottom shoulder
translate([0,0,-(totalHeight - bodyChamferRad)])
    rotate_extrude(convexity = 10, $fn = 100)
    translate([(bodyDia/2)-bodyChamferRad, 0, 0])
        circle(r = bodyChamferRad, $fn = 100);
        
//Bottom face of body
translate([0,0,-(totalHeight)])
    cylinder(
        h = bodyChamferRad, 
        d = (bodyDia - (2 * bodyChamferRad)),
        $fn = 100);

//Base cylinder
color("LightGrey")
translate([0,0,-(totalHeight + baseFilletRad)])
    cylinder(
        h = baseFilletRad, 
        d = (baseDia - (2 * baseFilletRad)),
        $fn = 100);

//Base shoulder
color("LightGrey")
translate([0,0,-(totalHeight)])
    rotate_extrude(convexity = 10, $fn = 100)
    translate([(baseDia/2)-baseFilletRad, 0, 0])
        circle(r = baseFilletRad, $fn = 100);
}
RenderCell();
