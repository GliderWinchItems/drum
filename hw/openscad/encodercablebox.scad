/* File: encodercablebox.scad
 * Box to hold perfboard with encoder cable-to-6-pin Telco cable w perfboard pullups.
 * Author: deh
 * Latest edit: 202009192020
 */
 
 $fn = 40;
 
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
 boxz = 12;
 
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
 
 frad = 4; // fillet radius
phole = 2.9; // Post screw hole diameter 

 module post_fillet(r,zzz)
 {
hrad = phole/2;     
hx = hrad * (1/sqrt(2));
prad = pstd/2;
alpha = acos((frad-hrad)/(frad+prad));
ofx = (pstd/2)*cos(alpha);
ofy = (pstd/2)*sin(alpha);     
     difference()
     {
         union()
         {
             translate([-hrad,hx,0])
            cube([frad-prad*cos(alpha),(frad+prad)*sin(alpha)-hx,zzz],center=false);
         }
         union()
         {
            translate([(prad+frad)*cos(alpha),(prad+frad)*sin(alpha),0])
              cylinder(d=frad*2,h=zzz,center=false);
         }         
     }
 }

 module postf(a,zz,ahole)
 {
     translate (a)
     {
        translate([0,0,boxz-zz])
        difference()
        {
            union()
            {   
                cylinder(d=pstd,h = zz,center=false);
                rotate([0,0,0])
                  post_fillet([0,0,0],zz);
                translate([0,0,zz])
                rotate([180,0,0])
                rotate([0,0,90])
                  post_fillet([0,0,0],zz);

            }
            union()
            {
                translate([0,0,zz-30])
                cylinder(d=ahole,h=60,center=false);
            }
        }
    }
}
/* Insert perf board */
edia = 4.6; // encoder cable dia     

 module mainbox()
 {
     difference()
     {
         union()
         {
             cube ([boxx,boxy,boxz],center=false);

             translate([    -paa,    -paa,0])
             rotate([0,0,-90])
                postf([0,0,0],pstz,phole);

             translate([boxx+paa,    -paa,0])
             rotate([0,0,0]) 
                postf([0,0,0],pstz,phole);

             translate([boxx+paa,boxy+paa,pstz])
             rotate([180,0,0])
                postf([0,0,0],pstz,phole);

             translate([    -paa,boxy+paa,0])
             rotate([0,0,180]) 
                postf([0,0,0],pstz,phole);
             
             
             
         }
         union()
         {
             // Main cutout
             translate([boxwall*0.5,boxwall*0.5,boxwall*0.5])
                cube([boxqx,boxqy,50],center=false);
             
             // Recess for perf board
             translate([boxwall*.25,boxwall*.25,boxz-4])
                cube([pbx,pby,50],center=false);
             
             
             // encoder cable
             spiq = boxfloor + spiofz;
             translate([boxx-5,(boxy-boxwall)/2+edia/2,boxz-edia/2])
                rotate([0,90,0])
                cylinder(d=edia,h=20, center=true);
             translate([boxx-5,(boxy-boxwall)/2+edia/2,boxz+5-edia/2])
                cube ([20,edia,10],center=true);
             
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
 edia1 = edia - 0.5;
 module cvrcabletab(a)
 {
    translate(a)
    translate([boxx-cvrdel/2,boxy/2,-cvrlh+6]) 
    {
     difference()
     {
         union()
         {
            cube([cvrdel,edia,4],center = true);
         }
         
         translate([0,0,-edia/2]) 
          rotate([0,90,0])
           cylinder(d=edia,h=20,center=true); 
     }
    }
 }
 module cvrcabletabW(a)
 {
     
    translate(a)
    translate([boxx-cvrdel/2+1.5,boxy/2,-cvrlh+6-.5]) 
    {
     difference()
     {
         union()
         {
            cube([1,edia,4],center = true);
//             translate([.5,0,-edia1*.5])
  //           ring(edia1);
         }
         union()
         {
            translate([0,0,-edia1/2]) 
             rotate([0,90,0])
              cylinder(d=edia1,h=20,center=true); 
         }
     }
    }
 }
 
 module ring(wdia)
 {
     sc = 0.75;
     
     
     translate([0,0,0])
     rotate([0,90,0])
     rotate_extrude(convexity = 10)
        translate([wdia*.5, 0, 0])
            polygon(points=[[0,0],[cos(60)*sc,0.5*sc],[cos(60)*sc,-0.5*sc]]);
 }
 //ring(edia);
 
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
module encodercablecvr()
{
   translate([0,0,0])
    cube([boxwall,edia,4],center=true);
} 

 
chole = 3.2;
 module cover(a)
 {
     translate(a)
     {
         difference()
         {
             union()
             {
                cube ([boxx,boxy,cvrz],center=false);
/*                cvrpost([    -paa,    -paa,0]);
                cvrpost([boxx+paa,    -paa,0]);
                cvrpost([boxx+paa,boxy+paa,0]);
                cvrpost([    -paa,boxy+paa,0]);
                 */
            translate([    -paa,    -paa,-2*cvrz])
             rotate([0,0,-90])
                postf([0,0,0],cvrz,chole);

             translate([boxx+paa,    -paa,-2*cvrz])
             rotate([0,0,0]) 
                postf([0,0,0],cvrz,chole);

             translate([boxx+paa,boxy+paa,3*cvrz])
             rotate([180,0,0])
                postf([0,0,0],cvrz,chole);

             translate([    -paa,boxy+paa,-2*cvrz])
             rotate([0,0,180]) 
                postf([0,0,0],cvrz,chole);  
  
cvrcabletab([0,0,0]);  
cvrcabletabW([0,0,0]);

             translate([boxx-cvrdel*.5,boxy*.5,-edia1*.5])
             ring(edia-1);


            
                
             }
             union()
             {
                // Recess in cover
                translate([boxwall*.25+cvrdel*.5,boxwall*.25+cvrdel*.5,0])
                    cube([pbx-cvrdel,pby-cvrdel,cvrz-1.5],center=false);
                 
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
