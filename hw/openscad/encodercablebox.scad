/* File: encodercablebox.scad
 * Box to hold perfboard with encoder cable-to-6-pin Telco cable w perfboard pullups.
 * Author: deh
 * Latest edit: 202009192020
 */
 
 $fn = 30;
 
 // Perfboard dimensions
 pbdel = 0.5;  // A little extra
 pbx  = 55 + pbdel; 
 pby  = 21 + pbdel;
 pbz  = 1.5;  // board thickness
 pbz1 = 4.2;  // bottom to top of wiring
 
// Box 
 boxwall = 6;
 boxfloor = 4;
 boxx = pbx + boxwall*0.5;
 boxy = pby + boxwall*0.5;
 boxz = 24;
 
 // Main box cutout
 boxqx = pbx - boxwall*0.5;
 boxqy = pby - boxwall*0.5;
 
 // Switch cable cutout
 swcofx = 37;
 swcofz =  2;
 swcx = 6;
 swcz = 20;
 
 // spi cable cutout
 spiofx = 42;
 spiofz =  2;
 spix = 20;
 spiz = 10;
 
 // Post height
 pstz = boxz; // Height
 pstd = 6;  // Diameter
 paa = 1.1; // Post center offset
 
 
 module post(a)
 {
     translate (a)
     {
        translate([0,0,boxz-pstz])
        difference()
        {
            union()
            {   
                cylinder(d=pstd,h = pstz,center=false);
            }
            union()
            {
                translate([0,0,pstz-30])
                cylinder(d=2.9,h=60,center=false);
            }
        }
    }
 }
 
 module mainbox()
 {
     difference()
     {
         union()
         {
             cube ([boxx,boxy,boxz],center=false);
             post([    -paa,    -paa,0]);
             post([boxx+paa,    -paa,0]);
             post([boxx+paa,boxy+paa,0]);
             post([    -paa,boxy+paa,0]);
         }
         union()
         {
             // Main cutout
             translate([boxwall*0.5,boxwall*0.5,boxwall*0.5])
                cube([boxqx,boxqy,50],center=false);
             
             // Recess for perf board
             translate([boxwall*.25,boxwall*.25,boxz-pbz])
                cube([pbx,pby,50],center=false);
             
             // Switch cable
             swcq = boxfloor + swcofz;
             translate([boxwall*.25+swcofx,-4,swcq])
                cube([swcx,20,swcz],center=false);
             
             // spi cable
//             spiq = boxfloor + spiofz;
//             translate([boxwall*.25+spiofx,boxy-8,spiq])
//                cube([spix,30,spiz],center=false);
             
         }
     }
 }
 cvrz   = 4;      // Cover thickness
 cvrofx = 10 + boxwall*.25; // pushbutton window offset
 cvrofy = 25 + boxwall*.25; // pushbutton window offset
 cvrx   = 60;    // Length of pushbutton window
 cvry   = 10;    // Width of pushbutton window
 cvrdel = 2.5;   // Overlap a edge
 
 module cvrpost(a)
 {
     translate (a)
     {
        translate([0,0,0])
        difference()
        {
            union()
            {   
                cylinder(d=pstd,h = cvrz,center=false);
            }
            union()
            {
                translate([0,0,-.5])
                cylinder(d=3.2,h=8,center=false);
            }
        }
    }
 }
 cvrw = 1.5;
 cvrlj = 2.0;
 cvrlh = cvrz + cvrlj;
 
 module cvrlip(a,r,l)
 {
      translate (a)
     {
        translate([0,0,cvrlj])
        difference()
        {
            union()
            {   
                translate([0,0,0])
                cube([l,cvrw,cvrlh],center=true);
            }
            union()
            {
            }
        }    
    }
 }
 
 // Toggle switch hole in cover
module togglehole(a)
{
   translate(a)
    cylinder(d = 6.4, h = 20, center=true);
} 
 
 module cover(a)
 {
     translate(a)
     {
         difference()
         {
             union()
             {
                cube ([boxx,boxy,cvrz],center=false);
                cvrpost([    -paa,    -paa,0]);
                cvrpost([boxx+paa,    -paa,0]);
                cvrpost([boxx+paa,boxy+paa,0]);
                cvrpost([    -paa,boxy+paa,0]);
                 
//cvrlip([boxx/2,-cvrw,0],[0,0,0],boxx-2*boxwall-8);                 
             }
             union()
             {
                // Recess in cover
//                translate([boxwall*.25+cvrdel*.5,boxwall*.25+cvrdel*.5,0])
//                    cube([pbx-cvrdel,pby-cvrdel,cvrz-1.5],center=false);
                 
                // pushbutton plate window
                 translate([cvrofx,cvrofy,-1])
                    cube([cvrx,cvry, 30],center=false);
    /*                
                // Holes for toggle switches
                 inc = 18; qx = (pbx - 3*inc)/2; qy = 13; 
                 togglehole([qx+(0*inc),qy,0]);
                 togglehole([qx+(1*inc),qy,0]);
                 togglehole([qx+(2*inc),qy,0]);                 
                 togglehole([qx+(3*inc),qy,0]);
    */                 
             }
         }
     }
 }

 
 pinx = 7.0;    // sw pin spacing x direction
 piny = 4.4;    // sw pin spacing y direction
 
 swplatex = 10.0 ;  // width
 swplatey = 8.0;
 sidey = 15.0;
 
 pwinx = 1.8;
 pwiny = 1.5;
 
 module pbhole(a)
 {
     translate(a)
     {
        cube([pwinx,pwiny,20],center=true);
     }
 }
 module pbholes(a)
 {
     translate(a)
     {
        pbhole([   0,   0, 0]);
        pbhole([pinx,piny, 0]);
        pbhole([pinx,   0, 0]);
        pbhole([   0,piny, 0]);
     }
 }
 
pbhx = 70;
pbhy = 20;
pbhz = 1.4;
 module pbplate(a)
 {
hofx = 6;
hofy = 8;     
incx = 16;
     
     translate(a)
     {
         difference()
         {
            cube([pbhx,pbhy,pbhz],center=false);
             
            union()
            {
             pbholes([hofx+0*incx,hofy,0]);
             pbholes([hofx+1*incx,hofy,0]);
             pbholes([hofx+2*incx,hofy,0]);
             pbholes([hofx+3*incx,hofy,0]);             
            }
        }
     }
 }
 
 mainbox();
 
 translate([0,-10,39])
 rotate([180,0,0])
   cover([0,0,35]);
 
 //pbplate([5,50,0]);
