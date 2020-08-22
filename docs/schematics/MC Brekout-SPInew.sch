<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="6" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="no" active="no"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="yes"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="OrgLBR" color="13" fill="1" visible="no" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="74xx-eu">
<description>&lt;b&gt;TTL Devices, 74xx Series with European Symbols&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Texas Instruments &lt;i&gt;TTL Data Book&lt;/i&gt;&amp;nbsp;&amp;nbsp;&amp;nbsp;Volume 1, 1996.
&lt;li&gt;TTL Data Book, Volume 2 , 1993
&lt;li&gt;National Seminconductor Databook 1990, ALS/LS Logic
&lt;li&gt;ttl 74er digital data dictionary, ECA Electronic + Acustic GmbH, ISBN 3-88109-032-0
&lt;li&gt;http://icmaster.com/ViewCompare.asp
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DIL16">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="10.16" y1="2.921" x2="-10.16" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="10.16" y1="2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="2.921" x2="-10.16" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="-10.16" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="1.016" x2="-10.16" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="-1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="-3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="-6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="16" x="-8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-10.541" y="-2.921" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-7.493" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SO16">
<description>&lt;b&gt;Small Outline package&lt;/b&gt; 150 mil</description>
<wire x1="4.699" y1="1.9558" x2="-4.699" y2="1.9558" width="0.1524" layer="51"/>
<wire x1="4.699" y1="-1.9558" x2="5.08" y2="-1.5748" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.5748" x2="-4.699" y2="1.9558" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.9558" x2="5.08" y2="1.5748" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.5748" x2="-4.699" y2="-1.9558" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="-1.9558" x2="4.699" y2="-1.9558" width="0.1524" layer="51"/>
<wire x1="5.08" y1="-1.5748" x2="5.08" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.5748" x2="-5.08" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.508" x2="-5.08" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.508" x2="-5.08" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.508" x2="-5.08" y2="-0.508" width="0.1524" layer="21" curve="-180"/>
<wire x1="-5.08" y1="-1.6002" x2="5.08" y2="-1.6002" width="0.0508" layer="21"/>
<smd name="1" x="-4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="16" x="-4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="15" x="-3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="-0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<text x="-3.81" y="-0.762" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.461" y="-1.905" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.889" y1="1.9558" x2="-0.381" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="-3.0988" x2="-4.191" y2="-1.9558" layer="51"/>
<rectangle x1="-3.429" y1="-3.0988" x2="-2.921" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="-3.0734" x2="-1.651" y2="-1.9304" layer="51"/>
<rectangle x1="-0.889" y1="-3.0988" x2="-0.381" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="1.9558" x2="-1.651" y2="3.0988" layer="51"/>
<rectangle x1="-3.429" y1="1.9558" x2="-2.921" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="1.9558" x2="-4.191" y2="3.0988" layer="51"/>
<rectangle x1="0.381" y1="-3.0988" x2="0.889" y2="-1.9558" layer="51"/>
<rectangle x1="1.651" y1="-3.0988" x2="2.159" y2="-1.9558" layer="51"/>
<rectangle x1="2.921" y1="-3.0988" x2="3.429" y2="-1.9558" layer="51"/>
<rectangle x1="4.191" y1="-3.0988" x2="4.699" y2="-1.9558" layer="51"/>
<rectangle x1="0.381" y1="1.9558" x2="0.889" y2="3.0988" layer="51"/>
<rectangle x1="1.651" y1="1.9558" x2="2.159" y2="3.0988" layer="51"/>
<rectangle x1="2.921" y1="1.9558" x2="3.429" y2="3.0988" layer="51"/>
<rectangle x1="4.191" y1="1.9558" x2="4.699" y2="3.0988" layer="51"/>
</package>
<package name="LCC20">
<description>&lt;b&gt;Leadless Chip Carrier&lt;/b&gt;&lt;p&gt; Ceramic Package</description>
<wire x1="-0.4001" y1="4.4" x2="-0.87" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="4.4" x2="-4.4" y2="3.3" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="4.3985" x2="0.4001" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.3985" x2="-0.8699" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="2.14" x2="-4.3985" y2="2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="-2.9401" y1="4.4" x2="-3.3" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.4" x2="0.4001" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.3985" x2="1.67" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="3.3" x2="-4.4" y2="2.9401" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="2.14" x2="-4.4" y2="1.6701" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="0.87" x2="-4.3985" y2="1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-0.4001" x2="-4.3985" y2="0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-1.6701" x2="-4.3985" y2="-0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="0.87" x2="-4.4" y2="0.4001" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-0.4001" x2="-4.4" y2="-0.87" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-2.9401" x2="-4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-4.4" y2="-4.4099" width="0.2032" layer="51"/>
<wire x1="2.14" y1="4.3985" x2="2.94" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="2.14" y1="4.4" x2="1.6701" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="4.4" x2="2.9401" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.4001" y1="-4.4" x2="0.87" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="-4.3985" x2="0.4001" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="0.87" y1="-4.3985" x2="1.67" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="2.9401" y1="-4.4" x2="4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.87" y1="-4.4" x2="-0.4001" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-1.6701" y1="-4.3985" x2="-0.8699" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.9401" y1="-4.3985" x2="-2.1399" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.14" y1="-4.4" x2="-1.6701" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-2.9401" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="0.4001" x2="4.4" y2="0.87" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="0.4001" x2="4.3985" y2="-0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="1.6701" x2="4.3985" y2="0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="2.9401" x2="4.4" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-0.87" x2="4.4" y2="-0.4001" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="-0.87" x2="4.3985" y2="-1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="-2.14" x2="4.3985" y2="-2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="-2.14" x2="4.4" y2="-1.6701" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-4.4" x2="4.4" y2="-2.9401" width="0.2032" layer="51"/>
<wire x1="-2.9401" y1="4.3985" x2="-2.1399" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.4" x2="-2.14" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="-2.9401" x2="-4.3985" y2="-2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="-1.6701" x2="-4.4" y2="-2.14" width="0.2032" layer="51"/>
<wire x1="1.6701" y1="-4.4" x2="2.14" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="2.14" y1="-4.3985" x2="2.94" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="4.3985" y1="2.9401" x2="4.3985" y2="2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="1.6701" x2="4.4" y2="2.14" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="4.4" x2="-4.4" y2="3.3" width="0.2032" layer="21"/>
<wire x1="-4.4" y1="-3.1941" x2="-4.4" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="-4.4" y1="-4.4" x2="-3.1941" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="3.1941" y1="-4.4" x2="4.4" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="4.4" y1="-4.4" x2="4.4" y2="-3.1941" width="0.2032" layer="21"/>
<wire x1="4.4" y1="3.1941" x2="4.4" y2="4.4" width="0.2032" layer="21"/>
<wire x1="4.4" y1="4.4" x2="3.1941" y2="4.4" width="0.2032" layer="21"/>
<smd name="2" x="-1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="1" x="0" y="3.8001" dx="0.8" dy="3.4" layer="1"/>
<smd name="3" x="-2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="4" x="-4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="5" x="-4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="6" x="-4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="7" x="-4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="8" x="-4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="9" x="-2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="10" x="-1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="11" x="0" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="12" x="1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="13" x="2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="14" x="4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="15" x="4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="16" x="4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="17" x="4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="18" x="4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="19" x="2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="20" x="1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<text x="-4.0051" y="6.065" size="1.778" layer="25">&gt;NAME</text>
<text x="-3.9751" y="-7.5601" size="1.778" layer="27">&gt;VALUE</text>
</package>
<package name="DIL14">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="8.89" y1="2.921" x2="-8.89" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-2.921" x2="8.89" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="8.89" y1="2.921" x2="8.89" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="2.921" x2="-8.89" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-2.921" x2="-8.89" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="1.016" x2="-8.89" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="0" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="0" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="-2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="-5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="-7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-9.271" y="-3.048" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-6.731" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SO14">
<description>&lt;b&gt;Small Outline package&lt;/b&gt; 150 mil</description>
<wire x1="4.064" y1="1.9558" x2="-4.064" y2="1.9558" width="0.1524" layer="51"/>
<wire x1="4.064" y1="-1.9558" x2="4.445" y2="-1.5748" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.445" y1="1.5748" x2="-4.064" y2="1.9558" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.064" y1="1.9558" x2="4.445" y2="1.5748" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.445" y1="-1.5748" x2="-4.064" y2="-1.9558" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.064" y1="-1.9558" x2="4.064" y2="-1.9558" width="0.1524" layer="51"/>
<wire x1="4.445" y1="-1.5748" x2="4.445" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.5748" x2="-4.445" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="0.508" x2="-4.445" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-0.508" x2="-4.445" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="0.508" x2="-4.445" y2="-0.508" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.445" y1="-1.6002" x2="4.445" y2="-1.6002" width="0.0508" layer="21"/>
<smd name="1" x="-3.81" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-3.81" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-2.54" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.27" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-2.54" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="-1.27" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="0" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="0" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="1.27" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="2.54" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="1.27" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="2.54" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.81" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="3.81" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<text x="-3.175" y="-0.762" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.826" y="-1.905" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.254" y1="1.9558" x2="0.254" y2="3.0988" layer="51"/>
<rectangle x1="-4.064" y1="-3.0988" x2="-3.556" y2="-1.9558" layer="51"/>
<rectangle x1="-2.794" y1="-3.0988" x2="-2.286" y2="-1.9558" layer="51"/>
<rectangle x1="-1.524" y1="-3.0734" x2="-1.016" y2="-1.9304" layer="51"/>
<rectangle x1="-0.254" y1="-3.0988" x2="0.254" y2="-1.9558" layer="51"/>
<rectangle x1="-1.524" y1="1.9558" x2="-1.016" y2="3.0988" layer="51"/>
<rectangle x1="-2.794" y1="1.9558" x2="-2.286" y2="3.0988" layer="51"/>
<rectangle x1="-4.064" y1="1.9558" x2="-3.556" y2="3.0988" layer="51"/>
<rectangle x1="1.016" y1="1.9558" x2="1.524" y2="3.0988" layer="51"/>
<rectangle x1="2.286" y1="1.9558" x2="2.794" y2="3.0988" layer="51"/>
<rectangle x1="3.556" y1="1.9558" x2="4.064" y2="3.0988" layer="51"/>
<rectangle x1="1.016" y1="-3.0988" x2="1.524" y2="-1.9558" layer="51"/>
<rectangle x1="2.286" y1="-3.0988" x2="2.794" y2="-1.9558" layer="51"/>
<rectangle x1="3.556" y1="-3.0988" x2="4.064" y2="-1.9558" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="74595">
<wire x1="-7.62" y1="-15.24" x2="7.62" y2="-15.24" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-15.24" x2="7.62" y2="12.7" width="0.4064" layer="94"/>
<wire x1="7.62" y1="12.7" x2="-7.62" y2="12.7" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="12.7" x2="-7.62" y2="-15.24" width="0.4064" layer="94"/>
<text x="-7.62" y="13.335" size="1.778" layer="95" font="vector">&gt;NAME</text>
<text x="-7.62" y="-17.78" size="1.778" layer="96">&gt;VALUE</text>
<pin name="QB" x="12.7" y="7.62" length="middle" direction="hiz" rot="R180"/>
<pin name="QC" x="12.7" y="5.08" length="middle" direction="hiz" rot="R180"/>
<pin name="QD" x="12.7" y="2.54" length="middle" direction="hiz" rot="R180"/>
<pin name="QE" x="12.7" y="0" length="middle" direction="hiz" rot="R180"/>
<pin name="QF" x="12.7" y="-2.54" length="middle" direction="hiz" rot="R180"/>
<pin name="QG" x="12.7" y="-5.08" length="middle" direction="hiz" rot="R180"/>
<pin name="QH" x="12.7" y="-7.62" length="middle" direction="hiz" rot="R180"/>
<pin name="SCL" x="-12.7" y="2.54" length="middle" direction="in" function="dot"/>
<pin name="SCK" x="-12.7" y="5.08" length="middle" direction="in" function="clk"/>
<pin name="RCK" x="-12.7" y="-2.54" length="middle" direction="in" function="clk"/>
<pin name="G" x="-12.7" y="-12.7" length="middle" direction="in" function="dot"/>
<pin name="SER" x="-12.7" y="10.16" length="middle" direction="in"/>
<pin name="QA" x="12.7" y="10.16" length="middle" direction="hiz" rot="R180"/>
<pin name="QH*" x="12.7" y="-12.7" length="middle" direction="hiz" rot="R180"/>
</symbol>
<symbol name="PWRN">
<text x="-0.635" y="-0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.905" y="-5.842" size="1.27" layer="95" rot="R90">GND</text>
<text x="1.905" y="2.54" size="1.27" layer="95" rot="R90">VCC</text>
<pin name="GND" x="0" y="-7.62" visible="pad" length="middle" direction="pwr" rot="R90"/>
<pin name="VCC" x="0" y="7.62" visible="pad" length="middle" direction="pwr" rot="R270"/>
</symbol>
<symbol name="74165">
<wire x1="-7.62" y1="-17.78" x2="7.62" y2="-17.78" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-17.78" x2="7.62" y2="17.78" width="0.4064" layer="94"/>
<wire x1="7.62" y1="17.78" x2="-7.62" y2="17.78" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="17.78" x2="-7.62" y2="-17.78" width="0.4064" layer="94"/>
<text x="-7.62" y="18.415" size="1.778" layer="95" font="vector">&gt;NAME</text>
<text x="-7.62" y="-20.32" size="1.778" layer="96">&gt;VALUE</text>
<pin name="SH/!LD" x="-12.7" y="-15.24" length="middle" direction="in"/>
<pin name="CLK" x="-12.7" y="-10.16" length="middle" direction="in" function="clk"/>
<pin name="E" x="-12.7" y="2.54" length="middle" direction="in"/>
<pin name="F" x="-12.7" y="0" length="middle" direction="in"/>
<pin name="G" x="-12.7" y="-2.54" length="middle" direction="in"/>
<pin name="H" x="-12.7" y="-5.08" length="middle" direction="in"/>
<pin name="!QH" x="12.7" y="5.08" length="middle" direction="out" rot="R180"/>
<pin name="QH" x="12.7" y="12.7" length="middle" direction="out" rot="R180"/>
<pin name="SER" x="-12.7" y="15.24" length="middle" direction="in"/>
<pin name="A" x="-12.7" y="12.7" length="middle" direction="in"/>
<pin name="B" x="-12.7" y="10.16" length="middle" direction="in"/>
<pin name="C" x="-12.7" y="7.62" length="middle" direction="in"/>
<pin name="D" x="-12.7" y="5.08" length="middle" direction="in"/>
<pin name="INH" x="-12.7" y="-12.7" length="middle" direction="in" function="clk"/>
</symbol>
<symbol name="7408">
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94" curve="-180"/>
<text x="2.54" y="3.175" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="I0" x="-7.62" y="2.54" visible="pad" length="middle" direction="in" swaplevel="1"/>
<pin name="I1" x="-7.62" y="-2.54" visible="pad" length="middle" direction="in" swaplevel="1"/>
<pin name="O" x="7.62" y="0" visible="pad" length="middle" direction="out" rot="R180"/>
</symbol>
<symbol name="7404">
<wire x1="-5.08" y1="5.08" x2="5.08" y2="0" width="0.4064" layer="94"/>
<wire x1="5.08" y1="0" x2="-5.08" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-5.08" y1="-5.08" x2="-5.08" y2="5.08" width="0.4064" layer="94"/>
<text x="2.54" y="3.175" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="I" x="-10.16" y="0" visible="pad" length="middle" direction="in"/>
<pin name="O" x="10.16" y="0" visible="pad" length="middle" direction="out" function="dot" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="74*595" prefix="IC">
<description>8-bit &lt;b&gt;SHIFT REGISTER&lt;/b&gt;, output latch</description>
<gates>
<gate name="A" symbol="74595" x="22.86" y="0"/>
<gate name="P" symbol="PWRN" x="-5.08" y="0" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL16">
<connects>
<connect gate="A" pin="G" pad="13"/>
<connect gate="A" pin="QA" pad="15"/>
<connect gate="A" pin="QB" pad="1"/>
<connect gate="A" pin="QC" pad="2"/>
<connect gate="A" pin="QD" pad="3"/>
<connect gate="A" pin="QE" pad="4"/>
<connect gate="A" pin="QF" pad="5"/>
<connect gate="A" pin="QG" pad="6"/>
<connect gate="A" pin="QH" pad="7"/>
<connect gate="A" pin="QH*" pad="9"/>
<connect gate="A" pin="RCK" pad="12"/>
<connect gate="A" pin="SCK" pad="11"/>
<connect gate="A" pin="SCL" pad="10"/>
<connect gate="A" pin="SER" pad="14"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
</technologies>
</device>
<device name="D" package="SO16">
<connects>
<connect gate="A" pin="G" pad="13"/>
<connect gate="A" pin="QA" pad="15"/>
<connect gate="A" pin="QB" pad="1"/>
<connect gate="A" pin="QC" pad="2"/>
<connect gate="A" pin="QD" pad="3"/>
<connect gate="A" pin="QE" pad="4"/>
<connect gate="A" pin="QF" pad="5"/>
<connect gate="A" pin="QG" pad="6"/>
<connect gate="A" pin="QH" pad="7"/>
<connect gate="A" pin="QH*" pad="9"/>
<connect gate="A" pin="RCK" pad="12"/>
<connect gate="A" pin="SCK" pad="11"/>
<connect gate="A" pin="SCL" pad="10"/>
<connect gate="A" pin="SER" pad="14"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="HC"/>
<technology name="HCT"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="G" pad="17"/>
<connect gate="A" pin="QA" pad="19"/>
<connect gate="A" pin="QB" pad="2"/>
<connect gate="A" pin="QC" pad="3"/>
<connect gate="A" pin="QD" pad="4"/>
<connect gate="A" pin="QE" pad="5"/>
<connect gate="A" pin="QF" pad="7"/>
<connect gate="A" pin="QG" pad="8"/>
<connect gate="A" pin="QH" pad="9"/>
<connect gate="A" pin="QH*" pad="12"/>
<connect gate="A" pin="RCK" pad="15"/>
<connect gate="A" pin="SCK" pad="14"/>
<connect gate="A" pin="SCL" pad="13"/>
<connect gate="A" pin="SER" pad="18"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="74*165" prefix="IC">
<description>8-bit parallel load &lt;b&gt;SHIFT REGISTER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="74165" x="20.32" y="0"/>
<gate name="P" symbol="PWRN" x="-5.08" y="0" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL16">
<connects>
<connect gate="A" pin="!QH" pad="7"/>
<connect gate="A" pin="A" pad="11"/>
<connect gate="A" pin="B" pad="12"/>
<connect gate="A" pin="C" pad="13"/>
<connect gate="A" pin="CLK" pad="2"/>
<connect gate="A" pin="D" pad="14"/>
<connect gate="A" pin="E" pad="3"/>
<connect gate="A" pin="F" pad="4"/>
<connect gate="A" pin="G" pad="5"/>
<connect gate="A" pin="H" pad="6"/>
<connect gate="A" pin="INH" pad="15"/>
<connect gate="A" pin="QH" pad="9"/>
<connect gate="A" pin="SER" pad="10"/>
<connect gate="A" pin="SH/!LD" pad="1"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name=""/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
</technologies>
</device>
<device name="D" package="SO16">
<connects>
<connect gate="A" pin="!QH" pad="7"/>
<connect gate="A" pin="A" pad="11"/>
<connect gate="A" pin="B" pad="12"/>
<connect gate="A" pin="C" pad="13"/>
<connect gate="A" pin="CLK" pad="2"/>
<connect gate="A" pin="D" pad="14"/>
<connect gate="A" pin="E" pad="3"/>
<connect gate="A" pin="F" pad="4"/>
<connect gate="A" pin="G" pad="5"/>
<connect gate="A" pin="H" pad="6"/>
<connect gate="A" pin="INH" pad="15"/>
<connect gate="A" pin="QH" pad="9"/>
<connect gate="A" pin="SER" pad="10"/>
<connect gate="A" pin="SH/!LD" pad="1"/>
<connect gate="P" pin="GND" pad="8"/>
<connect gate="P" pin="VCC" pad="16"/>
</connects>
<technologies>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="HC"/>
<technology name="LS"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="!QH" pad="9"/>
<connect gate="A" pin="A" pad="14"/>
<connect gate="A" pin="B" pad="15"/>
<connect gate="A" pin="C" pad="17"/>
<connect gate="A" pin="CLK" pad="3"/>
<connect gate="A" pin="D" pad="18"/>
<connect gate="A" pin="E" pad="4"/>
<connect gate="A" pin="F" pad="5"/>
<connect gate="A" pin="G" pad="7"/>
<connect gate="A" pin="H" pad="8"/>
<connect gate="A" pin="INH" pad="19"/>
<connect gate="A" pin="QH" pad="12"/>
<connect gate="A" pin="SER" pad="13"/>
<connect gate="A" pin="SH/!LD" pad="2"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="HC"/>
<technology name="LS"/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="74*08" prefix="IC">
<description>Quad 2-input &lt;b&gt;AND&lt;/b&gt; gate</description>
<gates>
<gate name="A" symbol="7408" x="20.32" y="0" swaplevel="1"/>
<gate name="B" symbol="7408" x="20.32" y="-12.7" swaplevel="1"/>
<gate name="C" symbol="7408" x="43.18" y="0" swaplevel="1"/>
<gate name="D" symbol="7408" x="43.18" y="-12.7" swaplevel="1"/>
<gate name="P" symbol="PWRN" x="2.54" y="-7.62" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL14">
<connects>
<connect gate="A" pin="I0" pad="1"/>
<connect gate="A" pin="I1" pad="2"/>
<connect gate="A" pin="O" pad="3"/>
<connect gate="B" pin="I0" pad="4"/>
<connect gate="B" pin="I1" pad="5"/>
<connect gate="B" pin="O" pad="6"/>
<connect gate="C" pin="I0" pad="9"/>
<connect gate="C" pin="I1" pad="10"/>
<connect gate="C" pin="O" pad="8"/>
<connect gate="D" pin="I0" pad="12"/>
<connect gate="D" pin="I1" pad="13"/>
<connect gate="D" pin="O" pad="11"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<technologies>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="D" package="SO14">
<connects>
<connect gate="A" pin="I0" pad="1"/>
<connect gate="A" pin="I1" pad="2"/>
<connect gate="A" pin="O" pad="3"/>
<connect gate="B" pin="I0" pad="4"/>
<connect gate="B" pin="I1" pad="5"/>
<connect gate="B" pin="O" pad="6"/>
<connect gate="C" pin="I0" pad="9"/>
<connect gate="C" pin="I1" pad="10"/>
<connect gate="C" pin="O" pad="8"/>
<connect gate="D" pin="I0" pad="12"/>
<connect gate="D" pin="I1" pad="13"/>
<connect gate="D" pin="O" pad="11"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<technologies>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="I0" pad="2"/>
<connect gate="A" pin="I1" pad="3"/>
<connect gate="A" pin="O" pad="4"/>
<connect gate="B" pin="I0" pad="6"/>
<connect gate="B" pin="I1" pad="8"/>
<connect gate="B" pin="O" pad="9"/>
<connect gate="C" pin="I0" pad="13"/>
<connect gate="C" pin="I1" pad="14"/>
<connect gate="C" pin="O" pad="12"/>
<connect gate="D" pin="I0" pad="18"/>
<connect gate="D" pin="I1" pad="19"/>
<connect gate="D" pin="O" pad="16"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="74*04" prefix="IC">
<description>Hex &lt;b&gt;INVERTER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="7404" x="17.78" y="0" swaplevel="1"/>
<gate name="B" symbol="7404" x="17.78" y="-12.7" swaplevel="1"/>
<gate name="C" symbol="7404" x="17.78" y="-25.4" swaplevel="1"/>
<gate name="D" symbol="7404" x="45.72" y="0" swaplevel="1"/>
<gate name="E" symbol="7404" x="45.72" y="-12.7" swaplevel="1"/>
<gate name="F" symbol="7404" x="45.72" y="-25.4" swaplevel="1"/>
<gate name="P" symbol="PWRN" x="-5.08" y="-10.16" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL14">
<connects>
<connect gate="A" pin="I" pad="1"/>
<connect gate="A" pin="O" pad="2"/>
<connect gate="B" pin="I" pad="3"/>
<connect gate="B" pin="O" pad="4"/>
<connect gate="C" pin="I" pad="5"/>
<connect gate="C" pin="O" pad="6"/>
<connect gate="D" pin="I" pad="9"/>
<connect gate="D" pin="O" pad="8"/>
<connect gate="E" pin="I" pad="11"/>
<connect gate="E" pin="O" pad="10"/>
<connect gate="F" pin="I" pad="13"/>
<connect gate="F" pin="O" pad="12"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="D" package="SO14">
<connects>
<connect gate="A" pin="I" pad="1"/>
<connect gate="A" pin="O" pad="2"/>
<connect gate="B" pin="I" pad="3"/>
<connect gate="B" pin="O" pad="4"/>
<connect gate="C" pin="I" pad="5"/>
<connect gate="C" pin="O" pad="6"/>
<connect gate="D" pin="I" pad="9"/>
<connect gate="D" pin="O" pad="8"/>
<connect gate="E" pin="I" pad="11"/>
<connect gate="E" pin="O" pad="10"/>
<connect gate="F" pin="I" pad="13"/>
<connect gate="F" pin="O" pad="12"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="I" pad="2"/>
<connect gate="A" pin="O" pad="3"/>
<connect gate="B" pin="I" pad="4"/>
<connect gate="B" pin="O" pad="6"/>
<connect gate="C" pin="I" pad="8"/>
<connect gate="C" pin="O" pad="9"/>
<connect gate="D" pin="I" pad="13"/>
<connect gate="D" pin="O" pad="12"/>
<connect gate="E" pin="I" pad="16"/>
<connect gate="E" pin="O" pad="14"/>
<connect gate="F" pin="I" pad="19"/>
<connect gate="F" pin="O" pad="18"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="transistor-small-signal">
<description>&lt;b&gt;Small Signal Transistors&lt;/b&gt;&lt;p&gt;
Packages from :&lt;br&gt;
www.infineon.com; &lt;br&gt;
www.semiconductors.com;&lt;br&gt;
www.irf.com&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="SOT54E">
<description>&lt;b&gt;SOT-54&lt;/b&gt;&lt;p&gt;
grid 2.54 mm</description>
<wire x1="-0.9692" y1="2.2098" x2="0.9692" y2="2.2098" width="0.1524" layer="51" curve="-47.3637"/>
<wire x1="-1.631" y1="-1.778" x2="-0.9689" y2="2.2098" width="0.1524" layer="21" curve="-113.782"/>
<wire x1="0.9689" y1="2.2098" x2="1.631" y2="-1.778" width="0.1524" layer="21" curve="-113.782"/>
<wire x1="-1.631" y1="-1.778" x2="1.631" y2="-1.778" width="0.1524" layer="21"/>
<pad name="D" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="G" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="S" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="3.175" y="0" size="1.27" layer="25" ratio="10">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="IGFNA">
<wire x1="-1.1176" y1="2.413" x2="-1.1176" y2="0" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.905" x2="0.5334" y2="1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="2.54" x2="2.54" y2="1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="0.508" y1="0" x2="1.778" y2="0.381" width="0.1524" layer="94"/>
<wire x1="1.778" y1="0.381" x2="1.778" y2="-0.381" width="0.1524" layer="94"/>
<wire x1="1.778" y1="-0.381" x2="0.508" y2="0" width="0.1524" layer="94"/>
<wire x1="0.508" y1="0" x2="1.143" y2="0" width="0.1524" layer="94"/>
<wire x1="1.143" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="0.508" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="-1.1176" y1="0" x2="-2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="-1.1176" y1="0" x2="-1.1176" y2="-2.54" width="0.254" layer="94"/>
<wire x1="1.651" y1="0.127" x2="1.143" y2="0" width="0.3048" layer="94"/>
<wire x1="1.143" y1="0" x2="1.651" y2="-0.127" width="0.3048" layer="94"/>
<circle x="2.54" y="-1.905" radius="0.254" width="0.4064" layer="94"/>
<text x="5.08" y="2.54" size="1.778" layer="95">&gt;NAME</text>
<text x="5.08" y="0" size="1.778" layer="96">&gt;VALUE</text>
<text x="1.27" y="2.413" size="0.8128" layer="93">D</text>
<text x="1.143" y="-3.175" size="0.8128" layer="93">S</text>
<text x="-2.413" y="0.381" size="0.8128" layer="93">G</text>
<rectangle x1="-0.254" y1="-2.54" x2="0.508" y2="-1.27" layer="94"/>
<rectangle x1="-0.254" y1="1.27" x2="0.508" y2="2.54" layer="94"/>
<rectangle x1="-0.254" y1="-0.889" x2="0.508" y2="0.889" layer="94"/>
<pin name="G" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="D" x="2.54" y="5.08" visible="off" length="short" direction="pas" rot="R270"/>
<pin name="S" x="2.54" y="-5.08" visible="off" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="BS170" prefix="Q">
<description>&lt;b&gt;N-CHANNEL MOS FET&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="IGFNA" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT54E">
<connects>
<connect gate="1" pin="D" pad="D"/>
<connect gate="1" pin="G" pad="G"/>
<connect gate="1" pin="S" pad="S"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="resistor">
<description>&lt;b&gt;Resistors, Capacitors, Inductors&lt;/b&gt;&lt;p&gt;
Based on the previous libraries:
&lt;ul&gt;
&lt;li&gt;r.lbr
&lt;li&gt;cap.lbr 
&lt;li&gt;cap-fe.lbr
&lt;li&gt;captant.lbr
&lt;li&gt;polcap.lbr
&lt;li&gt;ipc-smd.lbr
&lt;/ul&gt;
All SMD packages are defined according to the IPC specifications and  CECC&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
for Electrolyt Capacitors see also :&lt;p&gt;
www.bccomponents.com &lt;p&gt;
www.panasonic.com&lt;p&gt;
www.kemet.com&lt;p&gt;
&lt;p&gt;
for trimmer refence see : &lt;u&gt;www.electrospec-inc.com/cross_references/trimpotcrossref.asp&lt;/u&gt;&lt;p&gt;

&lt;map name="nav_main"&gt;
&lt;area shape="rect" coords="0,1,140,23" href="../military_specs.asp" title=""&gt;
&lt;area shape="rect" coords="0,24,140,51" href="../about.asp" title=""&gt;
&lt;area shape="rect" coords="1,52,140,77" href="../rfq.asp" title=""&gt;
&lt;area shape="rect" coords="0,78,139,103" href="../products.asp" title=""&gt;
&lt;area shape="rect" coords="1,102,138,128" href="../excess_inventory.asp" title=""&gt;
&lt;area shape="rect" coords="1,129,138,150" href="../edge.asp" title=""&gt;
&lt;area shape="rect" coords="1,151,139,178" href="../industry_links.asp" title=""&gt;
&lt;area shape="rect" coords="0,179,139,201" href="../comments.asp" title=""&gt;
&lt;area shape="rect" coords="1,203,138,231" href="../directory.asp" title=""&gt;
&lt;area shape="default" nohref&gt;
&lt;/map&gt;

&lt;html&gt;

&lt;title&gt;&lt;/title&gt;

 &lt;LINK REL="StyleSheet" TYPE="text/css" HREF="style-sheet.css"&gt;

&lt;body bgcolor="#ffffff" text="#000000" marginwidth="0" marginheight="0" topmargin="0" leftmargin="0"&gt;
&lt;table border=0 cellspacing=0 cellpadding=0 width="100%" cellpaddding=0 height="55%"&gt;
&lt;tr valign="top"&gt;

&lt;/td&gt;
&lt;! &lt;td width="10"&gt;&amp;nbsp;&lt;/td&gt;
&lt;td width="90%"&gt;

&lt;b&gt;&lt;font color="#0000FF" size="4"&gt;TRIM-POT CROSS REFERENCE&lt;/font&gt;&lt;/b&gt;
&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=2&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;RECTANGULAR MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BOURNS&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BI&amp;nbsp;TECH&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;DALE-VISHAY&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PHILIPS/MEPCO&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MURATA&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PANASONIC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;SPECTROL&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MILSPEC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;&lt;TD&gt;&amp;nbsp;&lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3 &gt;
      3005P&lt;BR&gt;
      3006P&lt;BR&gt;
      3006W&lt;BR&gt;
      3006Y&lt;BR&gt;
      3009P&lt;BR&gt;
      3009W&lt;BR&gt;
      3009Y&lt;BR&gt;
      3057J&lt;BR&gt;
      3057L&lt;BR&gt;
      3057P&lt;BR&gt;
      3057Y&lt;BR&gt;
      3059J&lt;BR&gt;
      3059L&lt;BR&gt;
      3059P&lt;BR&gt;
      3059Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      89P&lt;BR&gt;
      89W&lt;BR&gt;
      89X&lt;BR&gt;
      89PH&lt;BR&gt;
      76P&lt;BR&gt;
      89XH&lt;BR&gt;
      78SLT&lt;BR&gt;
      78L&amp;nbsp;ALT&lt;BR&gt;
      56P&amp;nbsp;ALT&lt;BR&gt;
      78P&amp;nbsp;ALT&lt;BR&gt;
      T8S&lt;BR&gt;
      78L&lt;BR&gt;
      56P&lt;BR&gt;
      78P&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      T18/784&lt;BR&gt;
      783&lt;BR&gt;
      781&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2199&lt;BR&gt;
      1697/1897&lt;BR&gt;
      1680/1880&lt;BR&gt;
      2187&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      8035EKP/CT20/RJ-20P&lt;BR&gt;
      -&lt;BR&gt;
      RJ-20X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      1211L&lt;BR&gt;
      8012EKQ&amp;nbsp;ALT&lt;BR&gt;
      8012EKR&amp;nbsp;ALT&lt;BR&gt;
      1211P&lt;BR&gt;
      8012EKJ&lt;BR&gt;
      8012EKL&lt;BR&gt;
      8012EKQ&lt;BR&gt;
      8012EKR&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      2101P&lt;BR&gt;
      2101W&lt;BR&gt;
      2101Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2102L&lt;BR&gt;
      2102S&lt;BR&gt;
      2102Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVMCOG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      43P&lt;BR&gt;
      43W&lt;BR&gt;
      43Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      40L&lt;BR&gt;
      40P&lt;BR&gt;
      40Y&lt;BR&gt;
      70Y-T602&lt;BR&gt;
      70L&lt;BR&gt;
      70P&lt;BR&gt;
      70Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SQUARE MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
   &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3250L&lt;BR&gt;
      3250P&lt;BR&gt;
      3250W&lt;BR&gt;
      3250X&lt;BR&gt;
      3252P&lt;BR&gt;
      3252W&lt;BR&gt;
      3252X&lt;BR&gt;
      3260P&lt;BR&gt;
      3260W&lt;BR&gt;
      3260X&lt;BR&gt;
      3262P&lt;BR&gt;
      3262W&lt;BR&gt;
      3262X&lt;BR&gt;
      3266P&lt;BR&gt;
      3266W&lt;BR&gt;
      3266X&lt;BR&gt;
      3290H&lt;BR&gt;
      3290P&lt;BR&gt;
      3290W&lt;BR&gt;
      3292P&lt;BR&gt;
      3292W&lt;BR&gt;
      3292X&lt;BR&gt;
      3296P&lt;BR&gt;
      3296W&lt;BR&gt;
      3296X&lt;BR&gt;
      3296Y&lt;BR&gt;
      3296Z&lt;BR&gt;
      3299P&lt;BR&gt;
      3299W&lt;BR&gt;
      3299X&lt;BR&gt;
      3299Y&lt;BR&gt;
      3299Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64P&amp;nbsp;ALT&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      64X&amp;nbsp;ALT&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66P&lt;BR&gt;
      66W&lt;BR&gt;
      66X&lt;BR&gt;
      67P&lt;BR&gt;
      67W&lt;BR&gt;
      67X&lt;BR&gt;
      67Y&lt;BR&gt;
      67Z&lt;BR&gt;
      68P&lt;BR&gt;
      68W&lt;BR&gt;
      68X&lt;BR&gt;
      67Y&amp;nbsp;ALT&lt;BR&gt;
      67Z&amp;nbsp;ALT&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      5050&lt;BR&gt;
      5091&lt;BR&gt;
      5080&lt;BR&gt;
      5087&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T63YB&lt;BR&gt;
      T63XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      5887&lt;BR&gt;
      5891&lt;BR&gt;
      5880&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T93Z&lt;BR&gt;
      T93YA&lt;BR&gt;
      T93XA&lt;BR&gt;
      T93YB&lt;BR&gt;
      T93XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKW&lt;BR&gt;
      8026EKM&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKB&lt;BR&gt;
      8026EKM&lt;BR&gt;
      1309X&lt;BR&gt;
      1309P&lt;BR&gt;
      1309W&lt;BR&gt;
      8024EKP&lt;BR&gt;
      8024EKW&lt;BR&gt;
      8024EKN&lt;BR&gt;
      RJ-9P/CT9P&lt;BR&gt;
      RJ-9W&lt;BR&gt;
      RJ-9X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3105P/3106P&lt;BR&gt;
      3105W/3106W&lt;BR&gt;
      3105X/3106X&lt;BR&gt;
      3105Y/3106Y&lt;BR&gt;
      3105Z/3105Z&lt;BR&gt;
      3102P&lt;BR&gt;
      3102W&lt;BR&gt;
      3102X&lt;BR&gt;
      3102Y&lt;BR&gt;
      3102Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMCBG&lt;BR&gt;
      EVMCCG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      55-1-X&lt;BR&gt;
      55-4-X&lt;BR&gt;
      55-3-X&lt;BR&gt;
      55-2-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      50-2-X&lt;BR&gt;
      50-4-X&lt;BR&gt;
      50-3-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      64Y&lt;BR&gt;
      64Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3323P&lt;BR&gt;
      3323S&lt;BR&gt;
      3323W&lt;BR&gt;
      3329H&lt;BR&gt;
      3329P&lt;BR&gt;
      3329W&lt;BR&gt;
      3339H&lt;BR&gt;
      3339P&lt;BR&gt;
      3339W&lt;BR&gt;
      3352E&lt;BR&gt;
      3352H&lt;BR&gt;
      3352K&lt;BR&gt;
      3352P&lt;BR&gt;
      3352T&lt;BR&gt;
      3352V&lt;BR&gt;
      3352W&lt;BR&gt;
      3362H&lt;BR&gt;
      3362M&lt;BR&gt;
      3362P&lt;BR&gt;
      3362R&lt;BR&gt;
      3362S&lt;BR&gt;
      3362U&lt;BR&gt;
      3362W&lt;BR&gt;
      3362X&lt;BR&gt;
      3386B&lt;BR&gt;
      3386C&lt;BR&gt;
      3386F&lt;BR&gt;
      3386H&lt;BR&gt;
      3386K&lt;BR&gt;
      3386M&lt;BR&gt;
      3386P&lt;BR&gt;
      3386S&lt;BR&gt;
      3386W&lt;BR&gt;
      3386X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      25P&lt;BR&gt;
      25S&lt;BR&gt;
      25RX&lt;BR&gt;
      82P&lt;BR&gt;
      82M&lt;BR&gt;
      82PA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      91E&lt;BR&gt;
      91X&lt;BR&gt;
      91T&lt;BR&gt;
      91B&lt;BR&gt;
      91A&lt;BR&gt;
      91V&lt;BR&gt;
      91W&lt;BR&gt;
      25W&lt;BR&gt;
      25V&lt;BR&gt;
      25P&lt;BR&gt;
      -&lt;BR&gt;
      25S&lt;BR&gt;
      25U&lt;BR&gt;
      25RX&lt;BR&gt;
      25X&lt;BR&gt;
      72XW&lt;BR&gt;
      72XL&lt;BR&gt;
      72PM&lt;BR&gt;
      72RX&lt;BR&gt;
      -&lt;BR&gt;
      72PX&lt;BR&gt;
      72P&lt;BR&gt;
      72RXW&lt;BR&gt;
      72RXL&lt;BR&gt;
      72X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T7YB&lt;BR&gt;
      T7YA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      TXD&lt;BR&gt;
      TYA&lt;BR&gt;
      TYP&lt;BR&gt;
      -&lt;BR&gt;
      TYD&lt;BR&gt;
      TX&lt;BR&gt;
      -&lt;BR&gt;
      150SX&lt;BR&gt;
      100SX&lt;BR&gt;
      102T&lt;BR&gt;
      101S&lt;BR&gt;
      190T&lt;BR&gt;
      150TX&lt;BR&gt;
      101&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      101SX&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ET6P&lt;BR&gt;
      ET6S&lt;BR&gt;
      ET6X&lt;BR&gt;
      RJ-6W/8014EMW&lt;BR&gt;
      RJ-6P/8014EMP&lt;BR&gt;
      RJ-6X/8014EMX&lt;BR&gt;
      TM7W&lt;BR&gt;
      TM7P&lt;BR&gt;
      TM7X&lt;BR&gt;
      -&lt;BR&gt;
      8017SMS&lt;BR&gt;
      -&lt;BR&gt;
      8017SMB&lt;BR&gt;
      8017SMA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      CT-6W&lt;BR&gt;
      CT-6H&lt;BR&gt;
      CT-6P&lt;BR&gt;
      CT-6R&lt;BR&gt;
      -&lt;BR&gt;
      CT-6V&lt;BR&gt;
      CT-6X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKV&lt;BR&gt;
      -&lt;BR&gt;
      8038EKX&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKP&lt;BR&gt;
      8038EKZ&lt;BR&gt;
      8038EKW&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3321H&lt;BR&gt;
      3321P&lt;BR&gt;
      3321N&lt;BR&gt;
      1102H&lt;BR&gt;
      1102P&lt;BR&gt;
      1102T&lt;BR&gt;
      RVA0911V304A&lt;BR&gt;
      -&lt;BR&gt;
      RVA0911H413A&lt;BR&gt;
      RVG0707V100A&lt;BR&gt;
      RVA0607V(H)306A&lt;BR&gt;
      RVA1214H213A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3104B&lt;BR&gt;
      3104C&lt;BR&gt;
      3104F&lt;BR&gt;
      3104H&lt;BR&gt;
      -&lt;BR&gt;
      3104M&lt;BR&gt;
      3104P&lt;BR&gt;
      3104S&lt;BR&gt;
      3104W&lt;BR&gt;
      3104X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      EVMQ0G&lt;BR&gt;
      EVMQIG&lt;BR&gt;
      EVMQ3G&lt;BR&gt;
      EVMS0G&lt;BR&gt;
      EVMQ0G&lt;BR&gt;
      EVMG0G&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMK4GA00B&lt;BR&gt;
      EVM30GA00B&lt;BR&gt;
      EVMK0GA00B&lt;BR&gt;
      EVM38GA00B&lt;BR&gt;
      EVMB6&lt;BR&gt;
      EVLQ0&lt;BR&gt;
      -&lt;BR&gt;
      EVMMSG&lt;BR&gt;
      EVMMBG&lt;BR&gt;
      EVMMAG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMMCS&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM0&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM3&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      62-3-1&lt;BR&gt;
      62-1-2&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67R&lt;BR&gt;
      -&lt;BR&gt;
      67P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67X&lt;BR&gt;
      63V&lt;BR&gt;
      63S&lt;BR&gt;
      63M&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63H&lt;BR&gt;
      63P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;&amp;nbsp;&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=3&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT color="#0000FF" SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SMD TRIM-POT CROSS REFERENCE&lt;/B&gt;&lt;/FONT&gt;
      &lt;P&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3224G&lt;BR&gt;
      3224J&lt;BR&gt;
      3224W&lt;BR&gt;
      3269P&lt;BR&gt;
      3269W&lt;BR&gt;
      3269X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      44G&lt;BR&gt;
      44J&lt;BR&gt;
      44W&lt;BR&gt;
      84P&lt;BR&gt;
      84W&lt;BR&gt;
      84X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST63Z&lt;BR&gt;
      ST63Y&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST5P&lt;BR&gt;
      ST5W&lt;BR&gt;
      ST5X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3314G&lt;BR&gt;
      3314J&lt;BR&gt;
      3364A/B&lt;BR&gt;
      3364C/D&lt;BR&gt;
      3364W/X&lt;BR&gt;
      3313G&lt;BR&gt;
      3313J&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      23B&lt;BR&gt;
      23A&lt;BR&gt;
      21X&lt;BR&gt;
      21W&lt;BR&gt;
      -&lt;BR&gt;
      22B&lt;BR&gt;
      22A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST5YL/ST53YL&lt;BR&gt;
      ST5YJ/5T53YJ&lt;BR&gt;
      ST-23A&lt;BR&gt;
      ST-22B&lt;BR&gt;
      ST-22&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST-4B&lt;BR&gt;
      ST-4A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST-3B&lt;BR&gt;
      ST-3A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVM-6YS&lt;BR&gt;
      EVM-1E&lt;BR&gt;
      EVM-1G&lt;BR&gt;
      EVM-1D&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      G4B&lt;BR&gt;
      G4A&lt;BR&gt;
      TR04-3S1&lt;BR&gt;
      TRG04-2S1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      DVR-43A&lt;BR&gt;
      CVR-42C&lt;BR&gt;
      CVR-42A/C&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;
&lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;ALT =&amp;nbsp;ALTERNATE&lt;/B&gt;&lt;/FONT&gt;
&lt;P&gt;

&amp;nbsp;
&lt;P&gt;
&lt;/td&gt;
&lt;/tr&gt;
&lt;/table&gt;
&lt;/BODY&gt;&lt;/HTML&gt;</description>
<packages>
<package name="R0402">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.762" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.032" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0603">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.889" y="0.889" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.889" y="-2.032" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0805">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.762" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.762" y="-2.286" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R0805W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.159" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R1005">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.762" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.032" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="R1206">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.397" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.397" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1206W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="21"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="21"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.651" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.651" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1210">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.27" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
</package>
<package name="R1210W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="21"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="21"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.651" y="1.524" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.651" y="-2.794" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="R2010">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-1.027" y1="1.245" x2="1.027" y2="1.245" width="0.1524" layer="21"/>
<wire x1="-1.002" y1="-1.245" x2="1.016" y2="-1.245" width="0.1524" layer="21"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-2.159" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.159" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2010W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="21"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="21"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.286" y="1.524" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.286" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2012">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.762" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.762" y="-2.286" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2012W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.286" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2512">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="1.473" x2="1.498" y2="1.473" width="0.1524" layer="21"/>
<wire x1="-1.473" y1="-1.473" x2="1.498" y2="-1.473" width="0.1524" layer="21"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.667" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.667" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R2512W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="21"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="21"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-2.794" y="1.778" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.794" y="-3.048" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R3216">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.397" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.397" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3216W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="21"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="21"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.524" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.524" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3225">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.397" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.397" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R3225W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="21"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="21"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.397" y="1.524" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.397" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R5025">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9" y1="1.245" x2="0.9" y2="1.245" width="0.1524" layer="21"/>
<wire x1="-0.875" y1="-1.245" x2="0.925" y2="-1.245" width="0.1524" layer="21"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-2.159" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.159" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R5025W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="21"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="21"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.286" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.286" y="-2.921" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="1.473" x2="1.498" y2="1.473" width="0.1524" layer="21"/>
<wire x1="-1.473" y1="-1.473" x2="1.498" y2="-1.473" width="0.1524" layer="21"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.794" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.794" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332W">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
chip, wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="21"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="21"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-2.921" y="1.778" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.921" y="-3.048" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M0805">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.016" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.016" y="-2.286" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M1206">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M1406">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="0.6858" y1="0.762" x2="-0.6858" y2="0.762" width="0.1524" layer="21"/>
<wire x1="0.6858" y1="-0.762" x2="-0.6858" y2="-0.762" width="0.1524" layer="21"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.651" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.651" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="M2012">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.016" y="1.016" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.016" y="-2.286" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M2309">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<wire x1="1.651" y1="1.1684" x2="-1.6764" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-1.1684" x2="-1.651" y2="-1.1684" width="0.1524" layer="21"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-2.794" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.794" y="-2.794" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M3216">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M3516">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="0.6858" y1="0.762" x2="-0.6858" y2="0.762" width="0.1524" layer="21"/>
<wire x1="0.6858" y1="-0.762" x2="-0.6858" y2="-0.762" width="0.1524" layer="21"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.651" y="1.143" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.651" y="-2.413" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="M5923">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<wire x1="1.651" y1="1.1684" x2="-1.6764" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-1.1684" x2="-1.651" y2="-1.1684" width="0.1524" layer="21"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-2.794" y="1.651" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.794" y="-2.794" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="0204/5">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0" x2="-2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.778" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.635" x2="1.778" y2="0.635" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.0066" y="1.1684" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-0.254" x2="-1.778" y2="0.254" layer="51"/>
<rectangle x1="1.778" y1="-0.254" x2="2.032" y2="0.254" layer="51"/>
</package>
<package name="0204/7">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="0207/10">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
</package>
<package name="0207/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 12 mm</description>
<wire x1="6.35" y1="0" x2="5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="4.445" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-4.445" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.3086" y2="0.3048" layer="21"/>
<rectangle x1="-5.3086" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
</package>
<package name="0207/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 15mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="5.715" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-5.715" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="5.715" y1="-0.3048" x2="6.5786" y2="0.3048" layer="21"/>
<rectangle x1="-6.5786" y1="-0.3048" x2="-5.715" y2="0.3048" layer="21"/>
</package>
<package name="0207/2V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="-0.381" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.6096" layer="21"/>
<wire x1="0.381" y1="0" x2="1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.27" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-0.0508" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0508" y="-2.2352" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/5V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-0.889" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.762" y1="0" x2="0.762" y2="0" width="0.6096" layer="21"/>
<wire x1="0.889" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<circle x="-2.54" y="0" radius="1.27" width="0.1016" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.143" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.143" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/7">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 7.5 mm</description>
<wire x1="-3.81" y1="0" x2="-3.429" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0" x2="3.81" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.5588" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-3.429" y1="-0.3048" x2="-3.175" y2="0.3048" layer="51"/>
<rectangle x1="3.175" y1="-0.3048" x2="3.429" y2="0.3048" layer="51"/>
</package>
<package name="0309/10">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 10mm</description>
<wire x1="-4.699" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="5.08" y1="0" x2="4.699" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.6228" y1="-0.3048" x2="-4.318" y2="0.3048" layer="51"/>
<rectangle x1="4.318" y1="-0.3048" x2="4.6228" y2="0.3048" layer="51"/>
</package>
<package name="0309/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.318" y1="-0.3048" x2="5.1816" y2="0.3048" layer="21"/>
<rectangle x1="-5.1816" y1="-0.3048" x2="-4.318" y2="0.3048" layer="21"/>
</package>
<package name="0411/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.762" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.762" layer="51"/>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.3594" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
<rectangle x1="5.08" y1="-0.381" x2="5.3594" y2="0.381" layer="21"/>
</package>
<package name="0411/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 15 mm</description>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-6.35" y2="0" width="0.762" layer="51"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.762" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="5.08" y1="-0.381" x2="6.477" y2="0.381" layer="21"/>
<rectangle x1="-6.477" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
</package>
<package name="0411V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 3.81 mm</description>
<wire x1="1.27" y1="0" x2="0.3048" y2="0" width="0.762" layer="51"/>
<wire x1="-1.5748" y1="0" x2="-2.54" y2="0" width="0.762" layer="51"/>
<circle x="-2.54" y="0" radius="2.032" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.9144" shape="octagon"/>
<text x="-0.508" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.5334" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.4732" y1="-0.381" x2="0.2032" y2="0.381" layer="21"/>
</package>
<package name="0414/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="1.905" x2="-5.842" y2="2.159" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-5.842" y2="-2.159" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="-2.159" x2="6.096" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="2.159" x2="6.096" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-6.096" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="2.159" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="2.032" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="-2.159" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="-4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.842" y1="2.159" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.842" y1="-2.159" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.905" x2="6.096" y2="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.5654" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="6.096" y1="-0.4064" x2="6.5024" y2="0.4064" layer="21"/>
<rectangle x1="-6.5024" y1="-0.4064" x2="-6.096" y2="0.4064" layer="21"/>
</package>
<package name="0414V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.159" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.381" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.381" y="-2.3622" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.2954" y2="0.4064" layer="21"/>
</package>
<package name="0617/17">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 17.5 mm</description>
<wire x1="-8.89" y1="0" x2="-8.636" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.636" y1="0" x2="8.89" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.016" shape="octagon"/>
<text x="-8.128" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.096" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-8.5344" y1="-0.4064" x2="-8.2296" y2="0.4064" layer="51"/>
<rectangle x1="8.2296" y1="-0.4064" x2="8.5344" y2="0.4064" layer="51"/>
</package>
<package name="0617/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 22.5 mm</description>
<wire x1="-10.287" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.287" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.255" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.1854" y1="-0.4064" x2="-8.255" y2="0.4064" layer="21"/>
<rectangle x1="8.255" y1="-0.4064" x2="10.1854" y2="0.4064" layer="21"/>
</package>
<package name="0617V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="3.048" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="0.635" y="1.4224" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.635" y="-2.6162" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.3208" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="0922/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 22.5 mm</description>
<wire x1="11.43" y1="0" x2="10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-11.43" y1="0" x2="-10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-10.16" y1="-4.191" x2="-10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="4.572" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="4.318" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="-4.572" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="-4.318" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="-8.636" y2="4.318" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="-8.636" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="9.779" y1="4.572" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="9.779" y1="-4.572" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-4.191" x2="10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-4.191" x2="-9.779" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-10.16" y1="4.191" x2="-9.779" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="9.779" y1="-4.572" x2="10.16" y2="-4.191" width="0.1524" layer="21" curve="90"/>
<wire x1="9.779" y1="4.572" x2="10.16" y2="4.191" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-10.16" y="5.1054" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.7188" y1="-0.4064" x2="-10.16" y2="0.4064" layer="51"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-10.16" y2="0.4064" layer="21"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.7188" y2="0.4064" layer="51"/>
</package>
<package name="P0613V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.254" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.254" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="P0613/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.032" x2="-6.223" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.032" x2="-6.223" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="-2.286" x2="6.477" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="2.286" x2="6.477" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.223" y1="2.286" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.159" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-6.223" y1="-2.286" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.159" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="-5.207" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="-5.207" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.223" y1="2.286" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-2.286" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="0.635" width="0.1524" layer="51"/>
<wire x1="6.477" y1="2.032" x2="6.477" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.032" x2="-6.477" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="2.032" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.477" y="2.6924" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-7.0358" y1="-0.4064" x2="-6.477" y2="0.4064" layer="51"/>
<rectangle x1="6.477" y1="-0.4064" x2="7.0358" y2="0.4064" layer="51"/>
</package>
<package name="P0817/22">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 22.5 mm</description>
<wire x1="-10.414" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="-3.429" x2="-8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="3.81" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="3.556" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="-3.81" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-3.556" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="-6.985" y2="3.556" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="-6.985" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.128" y1="3.81" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="8.128" y1="-3.81" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.429" x2="8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.414" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="3.429" x2="-8.128" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.509" y1="-3.429" x2="-8.128" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="8.128" y1="3.81" x2="8.509" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.128" y1="-3.81" x2="8.509" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="4.2164" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.604" y="-2.2606" size="1.27" layer="51" ratio="10" rot="R90">0817</text>
<rectangle x1="8.509" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-8.509" y2="0.4064" layer="21"/>
</package>
<package name="P0817V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 6.35 mm</description>
<wire x1="-3.81" y1="0" x2="-5.08" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="3.81" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="octagon"/>
<text x="-1.016" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.032" size="1.016" layer="21" ratio="12">0817</text>
<rectangle x1="-3.81" y1="-0.4064" x2="0" y2="0.4064" layer="21"/>
</package>
<package name="V234/12">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V234, grid 12.5 mm</description>
<wire x1="-4.953" y1="1.524" x2="-4.699" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.953" y1="-1.524" x2="-4.699" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="1.524" x2="-4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="-4.699" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.8128" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.016" shape="octagon"/>
<text x="-4.953" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.953" y1="-0.4064" x2="5.4102" y2="0.4064" layer="21"/>
<rectangle x1="-5.4102" y1="-0.4064" x2="-4.953" y2="0.4064" layer="21"/>
</package>
<package name="V235/17">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V235, grid 17.78 mm</description>
<wire x1="-6.731" y1="2.921" x2="6.731" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-7.112" y1="2.54" x2="-7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.921" x2="-6.731" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0" x2="7.874" y2="0" width="1.016" layer="51"/>
<wire x1="-7.874" y1="0" x2="-8.89" y2="0" width="1.016" layer="51"/>
<wire x1="-7.112" y1="-2.54" x2="-6.731" y2="-2.921" width="0.1524" layer="21" curve="90"/>
<wire x1="6.731" y1="2.921" x2="7.112" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="-2.921" x2="7.112" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-7.112" y1="2.54" x2="-6.731" y2="2.921" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-8.89" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.1938" shape="octagon"/>
<text x="-6.858" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.842" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="7.112" y1="-0.508" x2="7.747" y2="0.508" layer="21"/>
<rectangle x1="-7.747" y1="-0.508" x2="-7.112" y2="0.508" layer="21"/>
</package>
<package name="V526-0">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V526-0, grid 2.5 mm</description>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.413" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102AX">
<description>&lt;b&gt;Mini MELF 0102 Axial&lt;/b&gt;</description>
<circle x="0" y="0" radius="0.6" width="0" layer="51"/>
<circle x="0" y="0" radius="0.6" width="0" layer="52"/>
<smd name="1" x="0" y="0" dx="1.9" dy="1.9" layer="1" roundness="100"/>
<smd name="2" x="0" y="0" dx="1.9" dy="1.9" layer="16" roundness="100"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="1.3"/>
</package>
<package name="0922V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 7.5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.508" y="1.6764" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.508" y="-2.9972" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.54" size="1.016" layer="21" ratio="12">0922</text>
<rectangle x1="-3.81" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="MINI_MELF-0102R">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102W">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<smd name="2" x="0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204R">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204W">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207R">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<smd name="1" x="-2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<smd name="2" x="2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207W">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<smd name="1" x="-2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<smd name="2" x="2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<text x="-2.54" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="RDH/15">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type RDH, grid 15 mm</description>
<wire x1="-7.62" y1="0" x2="-6.858" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="3.048" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="2.794" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-3.048" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.794" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="-4.953" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="-4.953" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.667" x2="-6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-2.667" x2="6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.667" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="3.048" x2="6.477" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.667" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="6.096" y1="-3.048" x2="6.477" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.35" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="4.572" y="-1.7272" size="1.27" layer="51" ratio="10" rot="R90">RDH</text>
<rectangle x1="-6.7564" y1="-0.4064" x2="-6.4516" y2="0.4064" layer="51"/>
<rectangle x1="6.4516" y1="-0.4064" x2="6.7564" y2="0.4064" layer="51"/>
</package>
<package name="0204V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.508" layer="51"/>
<wire x1="-0.127" y1="0" x2="0.127" y2="0" width="0.508" layer="21"/>
<circle x="-1.27" y="0" radius="0.889" width="0.1524" layer="51"/>
<circle x="-1.27" y="0" radius="0.635" width="0.0508" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.1336" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0309V">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 2.5 mm</description>
<wire x1="1.27" y1="0" x2="0.635" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.635" y1="0" x2="-1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.524" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="0.254" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.254" y="-2.2098" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="0.254" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.635" y1="-0.3048" x2="-0.3302" y2="0.3048" layer="51"/>
<rectangle x1="-0.3302" y1="-0.3048" x2="0.254" y2="0.3048" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="R-US">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="R-US_" prefix="R" uservalue="yes">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="R-US" x="0" y="0"/>
</gates>
<devices>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R0805W" package="R0805W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1005" package="R1005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1206W" package="R1206W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R1210W" package="R1210W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2010W" package="R2010W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2012W" package="R2012W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R2512W" package="R2512W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3216W" package="R3216W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R3225W" package="R3225W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R5025W" package="R5025W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="R6332W" package="R6332W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M0805" package="M0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1206" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M1406" package="M1406">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2012" package="M2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M2309" package="M2309">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3216" package="M3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M3516" package="M3516">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="M5923" package="M5923">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/5" package="0204/5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/12" package="0207/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/15" package="0207/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/2V" package="0207/2V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/5V" package="0207/5V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0207/7" package="0207/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/10" package="0309/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/12" package="0309/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/12" package="0411/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/15" package="0411/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0411/3V" package="0411V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/15" package="0414/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0414/5V" package="0414V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/17" package="0617/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/22" package="0617/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0617/5V" package="0617V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922/22" package="0922/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/5V" package="P0613V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0613/15" package="P0613/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/22" package="P0817/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0817/7V" package="P0817V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V234/12" package="V234/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V235/17" package="V235/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="V526-0" package="V526-0">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102AX" package="MINI_MELF-0102AX">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0922V" package="0922V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102R" package="MINI_MELF-0102R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0102W" package="MINI_MELF-0102W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204R" package="MINI_MELF-0204R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0204W" package="MINI_MELF-0204W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207R" package="MINI_MELF-0207R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MELF0207W" package="MINI_MELF-0207W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="RDH/15" package="RDH/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0204/2V" package="0204V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0309/V" package="0309V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+5V">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="+12V">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.27" y1="-0.635" x2="0" y2="1.27" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="-0.635" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+12V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+5V" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+12V" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+12V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="buzzer">
<description>&lt;b&gt;Speakers and Buzzers&lt;/b&gt;&lt;p&gt;
&lt;ul&gt;Distributors:
&lt;li&gt;Buerklin
&lt;li&gt;Spoerle
&lt;li&gt;Schukat
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="EFBAA40D101">
<description>&lt;b&gt;Piezoelectric Acoustic Transducer&lt;/b&gt;&lt;p&gt;
Source: Panbasonic .. 2SC1685.pdf</description>
<circle x="0" y="0" radius="8.375" width="0.2032" layer="21"/>
<pad name="1" x="-5" y="0" drill="1.1" diameter="1.6764"/>
<pad name="2" x="5" y="0" drill="1.1" diameter="1.6764"/>
<text x="-3.175" y="3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="7.2" y1="-0.8" x2="8.375" y2="0.8" layer="21"/>
</package>
<package name="EFBRD22C41">
<description>&lt;b&gt;Piezoelectric Acoustic Transducer&lt;/b&gt;&lt;p&gt;
Source: Panbasonic .. 2SC1685.pdf</description>
<circle x="0" y="0" radius="10.9" width="0.2032" layer="21"/>
<pad name="1" x="-5" y="0" drill="1.1" diameter="1.6764"/>
<pad name="2" x="5" y="0" drill="1.1" diameter="1.6764"/>
<text x="-3.175" y="3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="9.675" y1="-0.8" x2="10.85" y2="0.8" layer="21"/>
</package>
<package name="EFBRD24C411">
<description>&lt;b&gt;Piezoelectric Acoustic Transducer&lt;/b&gt;&lt;p&gt;
Source: Panbasonic .. 2SC1685.pdf</description>
<circle x="0" y="0" radius="11.9" width="0.2032" layer="21"/>
<pad name="1" x="-5" y="0" drill="1.1" diameter="1.6764"/>
<pad name="2" x="5" y="0" drill="1.1" diameter="1.6764"/>
<text x="-3.175" y="3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="10.675" y1="-0.8" x2="11.85" y2="0.8" layer="21"/>
</package>
<package name="EFBRD22C413">
<description>&lt;b&gt;Piezoelectric Acoustic Transducer&lt;/b&gt;&lt;p&gt;
Source: Panbasonic .. 2SC1685.pdf</description>
<circle x="0" y="0" radius="10.9" width="0.2032" layer="21"/>
<pad name="1" x="-5" y="0" drill="1.1" diameter="1.6764"/>
<pad name="2" x="5" y="0" drill="1.1" diameter="1.6764"/>
<text x="-3.175" y="3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="9.675" y1="-0.8" x2="10.85" y2="0.8" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="B2P">
<wire x1="-1.27" y1="3.175" x2="0" y2="3.175" width="0.1524" layer="94"/>
<wire x1="0" y1="3.175" x2="0" y2="3.81" width="0.1524" layer="94"/>
<wire x1="0" y1="3.175" x2="0" y2="2.54" width="0.1524" layer="94"/>
<wire x1="0.635" y1="4.445" x2="0.635" y2="1.905" width="0.1524" layer="94"/>
<wire x1="0.635" y1="1.905" x2="1.905" y2="1.905" width="0.1524" layer="94"/>
<wire x1="1.905" y1="1.905" x2="1.905" y2="4.445" width="0.1524" layer="94"/>
<wire x1="1.905" y1="4.445" x2="0.635" y2="4.445" width="0.1524" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="3.175" width="0.1524" layer="94"/>
<wire x1="2.54" y1="3.175" x2="3.81" y2="3.175" width="0.1524" layer="94"/>
<wire x1="2.54" y1="3.175" x2="2.54" y2="2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="2.54" y2="1.27" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="1.27" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="1.27" x2="0" y2="1.27" width="0.254" layer="94"/>
<wire x1="5.08" y1="1.27" x2="5.08" y2="5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="5.08" x2="5.715" y2="5.08" width="0.254" layer="94"/>
<wire x1="5.715" y1="5.08" x2="5.715" y2="5.715" width="0.254" layer="94"/>
<wire x1="5.715" y1="5.715" x2="-3.175" y2="5.715" width="0.254" layer="94"/>
<wire x1="-3.175" y1="5.715" x2="-3.175" y2="5.08" width="0.254" layer="94"/>
<wire x1="-3.175" y1="5.08" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="1.27" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="5.08" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.27" x2="5.08" y2="1.27" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="2.54" y2="1.27" width="0.254" layer="94"/>
<text x="-2.54" y="6.35" size="1.778" layer="95">&gt;NAME</text>
<text x="6.35" y="0" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="-2.54" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="1" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="EFB?" prefix="SG">
<description>&lt;b&gt;Piezoelectric Acoustic Transducer&lt;/b&gt;&lt;p&gt;
Source: Panbasonic .. 2SC1685.pdf</description>
<gates>
<gate name="G$1" symbol="B2P" x="0" y="0"/>
</gates>
<devices>
<device name="AA40D101" package="EFBAA40D101">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="RD22C41" package="EFBRD22C41">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="RD24C411" package="EFBRD24C411">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="RD22C413" package="EFBRD22C413">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-molex">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="70543-20">
<description>&lt;b&gt;C-Grid SL Connector&lt;/b&gt;&lt;p&gt;
 vertical, 20 pin</description>
<wire x1="26.67" y1="-2.8575" x2="26.67" y2="2.8575" width="0.254" layer="21"/>
<wire x1="26.67" y1="2.8575" x2="-26.67" y2="2.8575" width="0.254" layer="21"/>
<wire x1="-26.67" y1="2.8575" x2="-26.67" y2="-2.8575" width="0.254" layer="21"/>
<wire x1="-26.67" y1="-2.8575" x2="-3.4925" y2="-2.8575" width="0.254" layer="21"/>
<wire x1="-3.4925" y1="-2.8575" x2="-3.4925" y2="-3.81" width="0.254" layer="21"/>
<wire x1="-3.4925" y1="-3.81" x2="3.4925" y2="-3.81" width="0.254" layer="21"/>
<wire x1="3.4925" y1="-3.81" x2="3.4925" y2="-2.8575" width="0.254" layer="21"/>
<wire x1="3.4925" y1="-2.8575" x2="26.67" y2="-2.8575" width="0.254" layer="21"/>
<wire x1="-26.035" y1="2.2225" x2="-26.035" y2="-2.2225" width="0.0508" layer="51"/>
<wire x1="26.035" y1="2.2225" x2="26.035" y2="-2.2225" width="0.0508" layer="51"/>
<wire x1="-26.035" y1="2.2225" x2="26.035" y2="2.2225" width="0.0508" layer="51"/>
<wire x1="-26.035" y1="-2.2225" x2="-2.8575" y2="-2.2225" width="0.0508" layer="51"/>
<wire x1="-2.8575" y1="-2.2225" x2="2.8575" y2="-2.2225" width="0.0508" layer="51"/>
<wire x1="2.8575" y1="-2.2225" x2="26.035" y2="-2.2225" width="0.0508" layer="51"/>
<wire x1="2.8575" y1="-3.175" x2="-2.8575" y2="-3.175" width="0.0508" layer="51"/>
<wire x1="-2.8575" y1="-2.2225" x2="-2.8575" y2="-3.175" width="0.0508" layer="51"/>
<wire x1="2.8575" y1="-2.2225" x2="2.8575" y2="-3.175" width="0.0508" layer="51"/>
<pad name="3" x="-19.05" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-21.59" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="1" x="-24.13" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-16.51" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-13.97" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="-11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="11" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="12" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="13" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="14" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="15" x="11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="16" x="13.97" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="17" x="16.51" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="18" x="19.05" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="19" x="21.59" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="20" x="24.13" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-27.305" y="-2.54" size="1.016" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-26.3525" y="3.4925" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="-25.7175" y="-1.5875" size="1.016" layer="51" ratio="10">1</text>
<rectangle x1="-21.8281" y1="-0.2381" x2="-21.3519" y2="0.2381" layer="51"/>
<rectangle x1="-24.3681" y1="-0.2381" x2="-23.8919" y2="0.2381" layer="51"/>
<rectangle x1="-16.7481" y1="-0.2381" x2="-16.2719" y2="0.2381" layer="51"/>
<rectangle x1="-19.2881" y1="-0.2381" x2="-18.8119" y2="0.2381" layer="51"/>
<rectangle x1="-11.6681" y1="-0.2381" x2="-11.1919" y2="0.2381" layer="51"/>
<rectangle x1="-14.2081" y1="-0.2381" x2="-13.7319" y2="0.2381" layer="51"/>
<rectangle x1="-6.5881" y1="-0.2381" x2="-6.1119" y2="0.2381" layer="51"/>
<rectangle x1="-9.1281" y1="-0.2381" x2="-8.6519" y2="0.2381" layer="51"/>
<rectangle x1="-1.5081" y1="-0.2381" x2="-1.0319" y2="0.2381" layer="51"/>
<rectangle x1="-4.0481" y1="-0.2381" x2="-3.5719" y2="0.2381" layer="51"/>
<rectangle x1="3.5719" y1="-0.2381" x2="4.0481" y2="0.2381" layer="51"/>
<rectangle x1="1.0319" y1="-0.2381" x2="1.5081" y2="0.2381" layer="51"/>
<rectangle x1="6.1119" y1="-0.2381" x2="6.5881" y2="0.2381" layer="51"/>
<rectangle x1="8.6519" y1="-0.2381" x2="9.1281" y2="0.2381" layer="51"/>
<rectangle x1="11.1919" y1="-0.2381" x2="11.6681" y2="0.2381" layer="51"/>
<rectangle x1="13.7319" y1="-0.2381" x2="14.2081" y2="0.2381" layer="51"/>
<rectangle x1="16.2719" y1="-0.2381" x2="16.7481" y2="0.2381" layer="51"/>
<rectangle x1="18.8119" y1="-0.2381" x2="19.2881" y2="0.2381" layer="51"/>
<rectangle x1="21.3519" y1="-0.2381" x2="21.8281" y2="0.2381" layer="51"/>
<rectangle x1="23.8919" y1="-0.2381" x2="24.3681" y2="0.2381" layer="51"/>
</package>
<package name="70553-20">
<description>&lt;b&gt;C-Grid SL Connector&lt;/b&gt;&lt;p&gt;
 right angle, 20 pin</description>
<wire x1="26.5113" y1="5.8738" x2="4.1275" y2="5.8738" width="0.254" layer="21"/>
<wire x1="4.1275" y1="5.8738" x2="-26.5113" y2="5.8738" width="0.254" layer="21"/>
<wire x1="-26.5113" y1="5.8738" x2="-26.5113" y2="-5.715" width="0.254" layer="21"/>
<wire x1="-26.5113" y1="-5.715" x2="26.5113" y2="-5.715" width="0.254" layer="21"/>
<wire x1="26.5113" y1="-5.715" x2="26.5113" y2="5.8738" width="0.254" layer="21"/>
<wire x1="-1.27" y1="2.54" x2="-1.27" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-1.27" y1="-3.175" x2="1.27" y2="-3.175" width="0.254" layer="21"/>
<wire x1="1.27" y1="-3.175" x2="1.27" y2="2.54" width="0.254" layer="21"/>
<wire x1="-4.1275" y1="5.8737" x2="-4.1275" y2="2.54" width="0.254" layer="21"/>
<wire x1="-4.1275" y1="2.54" x2="-3.175" y2="2.54" width="0.254" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-1.27" y2="2.54" width="0.254" layer="21"/>
<wire x1="1.27" y1="2.54" x2="3.175" y2="2.54" width="0.254" layer="21"/>
<wire x1="3.175" y1="2.54" x2="4.1275" y2="2.54" width="0.254" layer="21"/>
<wire x1="4.1275" y1="2.54" x2="4.1275" y2="5.8738" width="0.254" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.175" y2="3.175" width="0.254" layer="21"/>
<wire x1="-3.175" y1="3.175" x2="-1.905" y2="4.445" width="0.254" layer="21" curve="-90"/>
<wire x1="-1.905" y1="4.445" x2="1.905" y2="4.445" width="0.254" layer="21"/>
<wire x1="1.905" y1="4.445" x2="3.175" y2="3.175" width="0.254" layer="21" curve="-90"/>
<wire x1="3.175" y1="3.175" x2="3.175" y2="2.54" width="0.254" layer="21"/>
<wire x1="-24.765" y1="-6.35" x2="-23.495" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-23.495" y1="-6.35" x2="-22.225" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-23.495" y1="-6.35" x2="-23.495" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-23.495" y1="-7.6835" x2="-23.241" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-22.225" y1="-6.35" x2="-22.225" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-22.225" y1="-7.6835" x2="-22.479" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-22.479" y1="-7.9375" x2="-23.241" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-22.225" y1="-6.35" x2="-20.955" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-20.955" y1="-6.35" x2="-19.685" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-20.955" y1="-6.35" x2="-20.955" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-20.955" y1="-7.6835" x2="-20.701" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-19.685" y1="-6.35" x2="-19.685" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-19.685" y1="-7.6835" x2="-19.939" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-19.939" y1="-7.9375" x2="-20.701" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-19.685" y1="-6.35" x2="-18.415" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-18.415" y1="-6.35" x2="-17.145" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-18.415" y1="-6.35" x2="-18.415" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-18.415" y1="-7.6835" x2="-18.161" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-17.145" y1="-6.35" x2="-17.145" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-17.145" y1="-7.6835" x2="-17.399" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-17.399" y1="-7.9375" x2="-18.161" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-17.145" y1="-6.35" x2="-15.875" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-15.875" y1="-6.35" x2="-14.605" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-15.875" y1="-6.35" x2="-15.875" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-15.875" y1="-7.6835" x2="-15.621" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-14.605" y1="-6.35" x2="-14.605" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-14.605" y1="-7.6835" x2="-14.859" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-14.859" y1="-7.9375" x2="-15.621" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-14.605" y1="-6.35" x2="-13.335" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-13.335" y1="-6.35" x2="-12.065" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-13.335" y1="-6.35" x2="-13.335" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-13.335" y1="-7.6835" x2="-13.081" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-12.065" y1="-6.35" x2="-12.065" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-12.065" y1="-7.6835" x2="-12.319" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-12.319" y1="-7.9375" x2="-13.081" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-12.065" y1="-6.35" x2="-10.795" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-10.795" y1="-6.35" x2="-9.525" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-10.795" y1="-6.35" x2="-10.795" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-10.795" y1="-7.6835" x2="-10.541" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-9.525" y1="-6.35" x2="-9.525" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-9.525" y1="-7.6835" x2="-9.779" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-9.779" y1="-7.9375" x2="-10.541" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-9.525" y1="-6.35" x2="-8.255" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-8.255" y1="-6.35" x2="-6.985" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-8.255" y1="-6.35" x2="-8.255" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-8.255" y1="-7.6835" x2="-8.001" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-6.985" y1="-6.35" x2="-6.985" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-6.985" y1="-7.6835" x2="-7.239" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-7.239" y1="-7.9375" x2="-8.001" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-6.985" y1="-6.35" x2="-5.715" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-5.715" y1="-6.35" x2="-4.445" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-5.715" y1="-6.35" x2="-5.715" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-5.715" y1="-7.6835" x2="-5.461" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-4.445" y1="-6.35" x2="-4.445" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-4.445" y1="-7.6835" x2="-4.699" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-4.699" y1="-7.9375" x2="-5.461" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-4.445" y1="-6.35" x2="-3.175" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-3.175" y1="-6.35" x2="-1.905" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-3.175" y1="-6.35" x2="-3.175" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-3.175" y1="-7.6835" x2="-2.921" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="-1.905" y1="-6.35" x2="-1.905" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-1.905" y1="-7.6835" x2="-2.159" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-2.159" y1="-7.9375" x2="-2.921" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-1.905" y1="-6.35" x2="-0.635" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-0.635" y1="-6.35" x2="0.635" y2="-6.35" width="0.254" layer="51"/>
<wire x1="-0.635" y1="-6.35" x2="-0.635" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-0.635" y1="-7.6835" x2="-0.381" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="0.635" y1="-6.35" x2="0.635" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="0.635" y1="-7.6835" x2="0.381" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="0.381" y1="-7.9375" x2="-0.381" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="8.255" y1="-6.35" x2="12.065" y2="-6.35" width="0.254" layer="51"/>
<wire x1="12.065" y1="-6.35" x2="13.335" y2="-6.35" width="0.254" layer="51"/>
<wire x1="13.335" y1="-6.35" x2="14.605" y2="-6.35" width="0.127" layer="51"/>
<wire x1="14.605" y1="-6.35" x2="15.875" y2="-6.35" width="0.254" layer="51"/>
<wire x1="15.875" y1="-6.35" x2="17.145" y2="-6.35" width="0.127" layer="51"/>
<wire x1="17.145" y1="-6.35" x2="18.415" y2="-6.35" width="0.254" layer="51"/>
<wire x1="18.415" y1="-6.35" x2="19.685" y2="-6.35" width="0.127" layer="51"/>
<wire x1="19.685" y1="-6.35" x2="20.955" y2="-6.35" width="0.254" layer="51"/>
<wire x1="20.955" y1="-6.35" x2="22.225" y2="-6.35" width="0.127" layer="51"/>
<wire x1="22.225" y1="-6.35" x2="23.495" y2="-6.35" width="0.254" layer="51"/>
<wire x1="23.495" y1="-6.35" x2="24.765" y2="-6.35" width="0.127" layer="51"/>
<wire x1="-25.7175" y1="-5.715" x2="-25.7175" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="-24.765" y1="-6.35" x2="-24.765" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="-24.765" y1="-7.6835" x2="-25.019" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="-25.019" y1="-7.9375" x2="-25.7175" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="24.765" y1="-6.35" x2="24.765" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="24.765" y1="-7.6835" x2="25.019" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="25.019" y1="-7.9375" x2="25.7175" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="25.7175" y1="-7.9375" x2="25.7175" y2="-6.35" width="0.254" layer="51"/>
<wire x1="25.7175" y1="-6.35" x2="25.7175" y2="-5.715" width="0.254" layer="51"/>
<wire x1="24.765" y1="-6.35" x2="25.7175" y2="-6.35" width="0.254" layer="51"/>
<wire x1="0.635" y1="-6.35" x2="1.905" y2="-6.35" width="0.127" layer="51"/>
<wire x1="1.905" y1="-6.35" x2="3.175" y2="-6.35" width="0.254" layer="51"/>
<wire x1="1.905" y1="-6.35" x2="1.905" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="1.905" y1="-7.6835" x2="2.159" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="3.175" y1="-6.35" x2="3.175" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="3.175" y1="-7.6835" x2="2.921" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="2.921" y1="-7.9375" x2="2.159" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="5.715" y1="-6.35" x2="6.985" y2="-6.35" width="0.127" layer="51"/>
<wire x1="6.985" y1="-6.35" x2="8.255" y2="-6.35" width="0.254" layer="51"/>
<wire x1="6.985" y1="-6.35" x2="6.985" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="6.985" y1="-7.6835" x2="7.239" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="8.255" y1="-6.35" x2="8.255" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="8.255" y1="-7.6835" x2="8.001" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="8.001" y1="-7.9375" x2="7.239" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="3.175" y1="-6.35" x2="4.445" y2="-6.35" width="0.254" layer="51"/>
<wire x1="4.445" y1="-6.35" x2="5.715" y2="-6.35" width="0.254" layer="51"/>
<wire x1="4.445" y1="-6.35" x2="4.445" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="4.445" y1="-7.6835" x2="4.699" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="5.715" y1="-6.35" x2="5.715" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="5.715" y1="-7.6835" x2="5.461" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="5.461" y1="-7.9375" x2="4.699" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="9.525" y1="-6.35" x2="9.525" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="9.525" y1="-7.6835" x2="9.779" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="10.795" y1="-6.35" x2="10.795" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="10.795" y1="-7.6835" x2="10.541" y2="-7.9375" width="0.254" layer="51" curve="-90"/>
<wire x1="10.541" y1="-7.9375" x2="9.779" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="12.065" y1="-6.35" x2="12.065" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="12.065" y1="-7.6835" x2="12.319" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="12.319" y1="-7.9375" x2="13.081" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="13.081" y1="-7.9375" x2="13.335" y2="-7.6835" width="0.254" layer="51" curve="90"/>
<wire x1="13.335" y1="-7.6835" x2="13.335" y2="-6.35" width="0.254" layer="51"/>
<wire x1="19.685" y1="-6.35" x2="19.685" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="19.685" y1="-7.6835" x2="19.939" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="19.939" y1="-7.9375" x2="20.701" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="20.701" y1="-7.9375" x2="20.955" y2="-7.6835" width="0.254" layer="51" curve="90"/>
<wire x1="20.955" y1="-7.6835" x2="20.955" y2="-6.35" width="0.254" layer="51"/>
<wire x1="14.605" y1="-6.35" x2="14.605" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="14.605" y1="-7.6835" x2="14.859" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="14.859" y1="-7.9375" x2="15.621" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="15.621" y1="-7.9375" x2="15.875" y2="-7.6835" width="0.254" layer="51" curve="90"/>
<wire x1="15.875" y1="-7.6835" x2="15.875" y2="-6.35" width="0.254" layer="51"/>
<wire x1="17.145" y1="-6.35" x2="17.145" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="17.145" y1="-7.6835" x2="17.399" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="17.399" y1="-7.9375" x2="18.161" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="18.161" y1="-7.9375" x2="18.415" y2="-7.6835" width="0.254" layer="51" curve="90"/>
<wire x1="18.415" y1="-7.6835" x2="18.415" y2="-6.35" width="0.254" layer="51"/>
<wire x1="22.225" y1="-6.35" x2="22.225" y2="-7.6835" width="0.254" layer="51"/>
<wire x1="22.225" y1="-7.6835" x2="22.479" y2="-7.9375" width="0.254" layer="51" curve="90"/>
<wire x1="22.479" y1="-7.9375" x2="23.241" y2="-7.9375" width="0.254" layer="51"/>
<wire x1="23.241" y1="-7.9375" x2="23.495" y2="-7.6835" width="0.254" layer="51" curve="90"/>
<wire x1="23.495" y1="-7.6835" x2="23.495" y2="-6.35" width="0.254" layer="51"/>
<pad name="2" x="-21.59" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="1" x="-24.13" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="3" x="-19.05" y="-7.62" drill="1.0922" shape="long" rot="R270"/>
<pad name="4" x="-16.51" y="-7.62" drill="1.0922" shape="long" rot="R270"/>
<pad name="5" x="-13.97" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="6" x="-11.43" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="7" x="-8.89" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="8" x="-6.35" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="9" x="-3.81" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="10" x="-1.27" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="11" x="1.27" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="12" x="3.81" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="13" x="6.35" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="14" x="8.89" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="15" x="11.43" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="16" x="13.97" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="17" x="16.51" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="18" x="19.05" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="19" x="21.59" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<pad name="20" x="24.13" y="-7.62" drill="1.0922" shape="long" rot="R90"/>
<text x="-26.9875" y="-5.715" size="1.016" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="28.2575" y="-5.715" size="0.8128" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<rectangle x1="-24.4475" y1="-7.62" x2="-23.8125" y2="-6.35" layer="51"/>
<rectangle x1="-21.9075" y1="-7.62" x2="-21.2725" y2="-6.35" layer="51"/>
<rectangle x1="-19.3675" y1="-7.62" x2="-18.7325" y2="-6.35" layer="51"/>
<rectangle x1="-16.8275" y1="-7.62" x2="-16.1925" y2="-6.35" layer="51"/>
<rectangle x1="-14.2875" y1="-7.62" x2="-13.6525" y2="-6.35" layer="51"/>
<rectangle x1="-11.7475" y1="-7.62" x2="-11.1125" y2="-6.35" layer="51"/>
<rectangle x1="-9.2075" y1="-7.62" x2="-8.5725" y2="-6.35" layer="51"/>
<rectangle x1="-6.6675" y1="-7.62" x2="-6.0325" y2="-6.35" layer="51"/>
<rectangle x1="-4.1275" y1="-7.62" x2="-3.4925" y2="-6.35" layer="51"/>
<rectangle x1="-1.5875" y1="-7.62" x2="-0.9525" y2="-6.35" layer="51"/>
<rectangle x1="23.8125" y1="-7.62" x2="24.4475" y2="-6.35" layer="51"/>
<rectangle x1="0.9525" y1="-7.62" x2="1.5875" y2="-6.35" layer="51"/>
<rectangle x1="3.4925" y1="-7.62" x2="4.1275" y2="-6.35" layer="51"/>
<rectangle x1="6.0325" y1="-7.62" x2="6.6675" y2="-6.35" layer="51"/>
<rectangle x1="8.5725" y1="-7.62" x2="9.2075" y2="-6.35" layer="51"/>
<rectangle x1="11.1125" y1="-7.62" x2="11.7475" y2="-6.35" layer="51"/>
<rectangle x1="13.6525" y1="-7.62" x2="14.2875" y2="-6.35" layer="51"/>
<rectangle x1="16.1925" y1="-7.62" x2="16.8275" y2="-6.35" layer="51"/>
<rectangle x1="18.7325" y1="-7.62" x2="19.3675" y2="-6.35" layer="51"/>
<rectangle x1="21.2725" y1="-7.62" x2="21.9075" y2="-6.35" layer="51"/>
<polygon width="0.0508" layer="21">
<vertex x="-26.035" y="5.8737"/>
<vertex x="-25.5587" y="3.9688"/>
<vertex x="-25.0825" y="5.8737"/>
</polygon>
</package>
<package name="22-23-2121">
<description>.100" (2.54mm) Center Header - 12 Pin</description>
<wire x1="-15.24" y1="3.175" x2="15.24" y2="3.175" width="0.254" layer="21"/>
<wire x1="15.24" y1="3.175" x2="15.24" y2="1.27" width="0.254" layer="21"/>
<wire x1="15.24" y1="1.27" x2="15.24" y2="-3.175" width="0.254" layer="21"/>
<wire x1="15.24" y1="-3.175" x2="-15.24" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-15.24" y1="-3.175" x2="-15.24" y2="1.27" width="0.254" layer="21"/>
<wire x1="-15.24" y1="1.27" x2="-15.24" y2="3.175" width="0.254" layer="21"/>
<wire x1="-15.24" y1="1.27" x2="15.24" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-13.97" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-11.43" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="-8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="-6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="6" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="7" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="8" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="9" x="6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="10" x="8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="11" x="11.43" y="0" drill="1" shape="long" rot="R90"/>
<pad name="12" x="13.97" y="0" drill="1" shape="long" rot="R90"/>
<text x="-15.24" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-15.24" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="22-23-2041">
<description>.100" (2.54mm) Center Header - 4 Pin</description>
<wire x1="-5.08" y1="3.175" x2="5.08" y2="3.175" width="0.254" layer="21"/>
<wire x1="5.08" y1="3.175" x2="5.08" y2="1.27" width="0.254" layer="21"/>
<wire x1="5.08" y1="1.27" x2="5.08" y2="-3.175" width="0.254" layer="21"/>
<wire x1="5.08" y1="-3.175" x2="-5.08" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-5.08" y1="-3.175" x2="-5.08" y2="1.27" width="0.254" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="3.175" width="0.254" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="5.08" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<text x="-5.08" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="MV">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C-GRID-20" prefix="X">
<description>&lt;b&gt;CONNECTOR&lt;/b&gt;&lt;p&gt;
wire to board 2.54 mm (0.100") pitch header</description>
<gates>
<gate name="-1" symbol="MV" x="2.54" y="0" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="2.54" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="2.54" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="2.54" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="2.54" y="-10.16" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="2.54" y="-12.7" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="2.54" y="-15.24" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="2.54" y="-17.78" addlevel="always" swaplevel="1"/>
<gate name="-9" symbol="M" x="2.54" y="-20.32" addlevel="always" swaplevel="1"/>
<gate name="-10" symbol="M" x="2.54" y="-22.86" addlevel="always" swaplevel="1"/>
<gate name="-11" symbol="M" x="2.54" y="-25.4" addlevel="always" swaplevel="1"/>
<gate name="-12" symbol="M" x="2.54" y="-27.94" addlevel="always" swaplevel="1"/>
<gate name="-13" symbol="M" x="2.54" y="-30.48" addlevel="always" swaplevel="1"/>
<gate name="-14" symbol="M" x="2.54" y="-33.02" addlevel="always" swaplevel="1"/>
<gate name="-15" symbol="M" x="2.54" y="-35.56" addlevel="always" swaplevel="1"/>
<gate name="-16" symbol="M" x="2.54" y="-38.1" addlevel="always" swaplevel="1"/>
<gate name="-17" symbol="M" x="2.54" y="-40.64" addlevel="always" swaplevel="1"/>
<gate name="-18" symbol="M" x="2.54" y="-43.18" addlevel="always" swaplevel="1"/>
<gate name="-19" symbol="M" x="2.54" y="-45.72" addlevel="always" swaplevel="1"/>
<gate name="-20" symbol="M" x="2.54" y="-48.26" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="-70543" package="70543-20">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-12" pin="S" pad="12"/>
<connect gate="-13" pin="S" pad="13"/>
<connect gate="-14" pin="S" pad="14"/>
<connect gate="-15" pin="S" pad="15"/>
<connect gate="-16" pin="S" pad="16"/>
<connect gate="-17" pin="S" pad="17"/>
<connect gate="-18" pin="S" pad="18"/>
<connect gate="-19" pin="S" pad="19"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-20" pin="S" pad="20"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="-70553" package="70553-20">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-12" pin="S" pad="12"/>
<connect gate="-13" pin="S" pad="13"/>
<connect gate="-14" pin="S" pad="14"/>
<connect gate="-15" pin="S" pad="15"/>
<connect gate="-16" pin="S" pad="16"/>
<connect gate="-17" pin="S" pad="17"/>
<connect gate="-18" pin="S" pad="18"/>
<connect gate="-19" pin="S" pad="19"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-20" pin="S" pad="20"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2121" prefix="X">
<description>.100" (2.54mm) Center Header - 12 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="12.7" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="10.16" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-9" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-10" symbol="M" x="0" y="-10.16" addlevel="always" swaplevel="1"/>
<gate name="-11" symbol="M" x="0" y="-12.7" addlevel="always" swaplevel="1"/>
<gate name="-12" symbol="M" x="0" y="-15.24" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2121">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-12" pin="S" pad="12"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2121" constant="no"/>
<attribute name="OC_FARNELL" value="1462935" constant="no"/>
<attribute name="OC_NEWARK" value="56H0456" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2041" prefix="X">
<description>.100" (2.54mm) Center Header - 4 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2041">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2041" constant="no"/>
<attribute name="OC_FARNELL" value="1462920" constant="no"/>
<attribute name="OC_NEWARK" value="38C0355" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="deh">
<packages>
<package name="SMT0805">
<wire x1="0" y1="-0.6" x2="0.1" y2="-0.6" width="0.127" layer="21"/>
<wire x1="0.1" y1="-0.6" x2="1" y2="-0.6" width="0.127" layer="21"/>
<wire x1="1" y1="-0.6" x2="1" y2="0.6" width="0.127" layer="21"/>
<wire x1="1" y1="0.6" x2="-1" y2="0.6" width="0.127" layer="21"/>
<wire x1="-1" y1="0.6" x2="-1" y2="-0.6" width="0.127" layer="21"/>
<wire x1="-1" y1="-0.6" x2="0.1" y2="-0.6" width="0.127" layer="21"/>
<smd name="P$1" x="-1" y="0" dx="0.635" dy="1.27" layer="1"/>
<smd name="P$2" x="1" y="0" dx="0.635" dy="1.27" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="CAP">
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="-1.016" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1" x2="2.4892" y2="-1.8541" width="0.254" layer="94" curve="-37.874808" cap="flat"/>
<wire x1="-2.4668" y1="-1.8504" x2="0" y2="-1.0161" width="0.254" layer="94" curve="-37.373024" cap="flat"/>
<text x="2.54" y="1.27" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SMT0805" prefix="C" uservalue="yes">
<gates>
<gate name="G$1" symbol="CAP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SMT0805">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IC2" library="74xx-eu" deviceset="74*165" device="N" technology="HC" value="74HC165N"/>
<part name="IC3" library="74xx-eu" deviceset="74*165" device="N" technology="HC" value="74HC165N"/>
<part name="IC4" library="74xx-eu" deviceset="74*595" device="N" technology="HC"/>
<part name="Q1" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="R1" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="IC5" library="74xx-eu" deviceset="74*595" device="N" technology="HC"/>
<part name="R2" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R3" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R4" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R5" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R6" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R7" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R8" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R9" library="resistor" deviceset="R-US_" device="V234/12" value="47"/>
<part name="R10" library="resistor" deviceset="R-US_" device="V234/12" value="47"/>
<part name="R11" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="R12" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="R13" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="R14" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="R15" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R16" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R33" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R34" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R35" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R36" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R37" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R38" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R39" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R40" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R41" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R42" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R43" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R44" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R45" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R46" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R47" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R48" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R49" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="P+1" library="supply1" deviceset="+5V" device=""/>
<part name="R50" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R51" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R52" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R53" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="IC6" library="74xx-eu" deviceset="74*08" device="N" technology="HCT"/>
<part name="IC7" library="74xx-eu" deviceset="74*04" device="N" technology="HCT"/>
<part name="Q2" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q3" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q4" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q5" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q6" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q7" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q8" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="R17" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R18" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R19" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R20" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R21" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R22" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R23" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R24" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R25" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R26" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R27" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R28" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R29" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R30" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R31" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="R32" library="resistor" deviceset="R-US_" device="V234/12" value="390"/>
<part name="Q9" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q10" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q11" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q12" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q13" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q14" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q15" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q16" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q17" library="transistor-small-signal" deviceset="BS170" device=""/>
<part name="Q18" library="transistor-small-signal" deviceset="BS170" device="" value="IRFXXXX"/>
<part name="R54" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R55" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="R56" library="resistor" deviceset="R-US_" device="V234/12" value="1K"/>
<part name="SG1" library="buzzer" deviceset="EFB?" device="AA40D101"/>
<part name="P+2" library="supply1" deviceset="+12V" device=""/>
<part name="R57" library="resistor" deviceset="R-US_" device="V234/12" value="18K"/>
<part name="GND2" library="supply1" deviceset="GND" device=""/>
<part name="J9" library="con-molex" deviceset="22-23-2121" device="" value="16 Pin"/>
<part name="P+3" library="supply1" deviceset="+12V" device=""/>
<part name="P+4" library="supply1" deviceset="+12V" device=""/>
<part name="R58" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="C12" library="deh" deviceset="SMT0805" device="" value="0.1"/>
<part name="R59" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
<part name="GND3" library="supply1" deviceset="GND" device=""/>
<part name="P6" library="con-molex" deviceset="22-23-2041" device="" value="Molex 4 Pin"/>
<part name="J8" library="con-molex" deviceset="C-GRID-20" device="-70543" value="20 Pin"/>
<part name="GND1" library="supply1" deviceset="GND" device=""/>
<part name="GND4" library="supply1" deviceset="GND" device=""/>
<part name="GND5" library="supply1" deviceset="GND" device=""/>
<part name="R60" library="resistor" deviceset="R-US_" device="V234/12" value="330"/>
<part name="R61" library="resistor" deviceset="R-US_" device="V234/12" value="TBD"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="192.024" y="79.248" size="2.54" layer="97">+</text>
<text x="295.402" y="96.266" size="1.778" layer="91">Unused</text>
<text x="277.876" y="85.852" size="1.27" layer="91">Independent wire
to battery return</text>
<text x="307.848" y="151.384" size="1.778" layer="91">To State
LED Board</text>
<text x="309.626" y="215.138" size="1.778" layer="91">To Lighted
Pushbuttons</text>
<text x="280.924" y="64.008" size="1.778" layer="91">Molex 4 Pin</text>
<text x="188.468" y="89.916" size="1.778" layer="91" rot="R180">Beeper</text>
<text x="168.148" y="197.104" size="1.016" layer="91">This resistor 
is not actually
needed</text>
<text x="30.988" y="166.624" size="1.778" layer="91">To Control
Panel Switches</text>
<text x="307.848" y="169.164" size="1.778" layer="91">To State
LED Board</text>
<text x="277.368" y="159.004" size="2.286" layer="91">Unused positions
need not be 
populated</text>
<text x="250.698" y="211.328" size="1.27" layer="91">ARM (Red)</text>
<text x="250.952" y="205.994" size="1.27" layer="91">Prep/Retrieve
(Yellow)</text>
</plain>
<instances>
<instance part="IC2" gate="A" x="137.16" y="180.34"/>
<instance part="IC3" gate="A" x="137.16" y="134.62"/>
<instance part="IC4" gate="A" x="180.34" y="175.26"/>
<instance part="Q1" gate="1" x="200.66" y="208.28" smashed="yes" rot="MR90">
<attribute name="NAME" x="196.85" y="206.502" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="197.104" y="213.106" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R1" gate="G$1" x="200.66" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="199.898" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="199.898" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IC5" gate="A" x="180.34" y="114.3"/>
<instance part="R2" gate="G$1" x="210.82" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="210.058" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="210.312" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R3" gate="G$1" x="220.98" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="220.218" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="220.472" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R4" gate="G$1" x="231.14" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="230.378" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="230.632" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R5" gate="G$1" x="241.3" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="240.538" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="240.792" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R6" gate="G$1" x="251.46" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="250.698" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="250.952" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R7" gate="G$1" x="261.62" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="260.858" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="261.112" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R8" gate="G$1" x="271.78" y="160.02" smashed="yes" rot="R90">
<attribute name="NAME" x="271.018" y="164.3634" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="271.526" y="156.972" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R9" gate="G$1" x="287.02" y="210.82" smashed="yes">
<attribute name="NAME" x="283.718" y="212.8774" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="291.592" y="212.852" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R10" gate="G$1" x="287.02" y="205.74" smashed="yes">
<attribute name="NAME" x="283.718" y="207.7974" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="291.592" y="207.772" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R11" gate="G$1" x="271.78" y="200.66" smashed="yes">
<attribute name="NAME" x="268.478" y="202.7174" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="276.352" y="202.692" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R12" gate="G$1" x="274.32" y="195.58" smashed="yes">
<attribute name="NAME" x="271.018" y="197.6374" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="278.892" y="197.612" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R13" gate="G$1" x="276.86" y="190.5" smashed="yes">
<attribute name="NAME" x="273.558" y="192.5574" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="281.432" y="192.532" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R14" gate="G$1" x="279.4" y="185.42" smashed="yes">
<attribute name="NAME" x="276.098" y="187.4774" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="283.972" y="187.452" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R15" gate="G$1" x="284.48" y="180.34" smashed="yes">
<attribute name="NAME" x="281.178" y="182.3974" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="289.052" y="182.372" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R16" gate="G$1" x="287.02" y="175.26" smashed="yes">
<attribute name="NAME" x="283.718" y="177.3174" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="291.592" y="177.292" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R33" gate="G$1" x="165.1" y="198.12" smashed="yes" rot="R90">
<attribute name="NAME" x="163.83" y="201.7014" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="170.688" y="195.072" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R34" gate="G$1" x="104.14" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="102.87" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="108.712" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R35" gate="G$1" x="96.52" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="95.25" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="101.092" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R36" gate="G$1" x="88.9" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="87.63" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="93.472" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R37" gate="G$1" x="81.28" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="80.01" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="85.852" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R38" gate="G$1" x="73.66" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="72.39" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="78.232" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R39" gate="G$1" x="66.04" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="64.77" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="70.612" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R40" gate="G$1" x="58.42" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="57.15" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="62.992" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R41" gate="G$1" x="50.8" y="200.66" smashed="yes" rot="R90">
<attribute name="NAME" x="49.53" y="204.2414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="55.372" y="197.866" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R42" gate="G$1" x="104.14" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="102.87" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="108.712" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R43" gate="G$1" x="96.52" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="95.25" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="101.092" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R44" gate="G$1" x="88.9" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="87.63" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="93.472" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R45" gate="G$1" x="81.28" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="80.01" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="85.852" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R46" gate="G$1" x="73.66" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="72.39" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="78.232" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R47" gate="G$1" x="66.04" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="64.77" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="70.612" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R48" gate="G$1" x="58.42" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="57.15" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="62.992" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R49" gate="G$1" x="50.8" y="154.94" smashed="yes" rot="R90">
<attribute name="NAME" x="49.53" y="158.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="55.372" y="151.892" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="P+1" gate="1" x="165.1" y="220.98" smashed="yes">
<attribute name="VALUE" x="171.196" y="220.98" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R50" gate="G$1" x="78.74" y="116.84" smashed="yes" rot="R90">
<attribute name="NAME" x="77.47" y="120.4214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="76.708" y="113.792" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R51" gate="G$1" x="71.12" y="116.84" smashed="yes" rot="R90">
<attribute name="NAME" x="69.85" y="120.4214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="69.088" y="113.792" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R52" gate="G$1" x="60.96" y="116.84" smashed="yes" rot="R90">
<attribute name="NAME" x="59.69" y="120.4214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="58.928" y="113.792" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R53" gate="G$1" x="50.8" y="116.84" smashed="yes" rot="R90">
<attribute name="NAME" x="49.53" y="120.4214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="48.768" y="113.792" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IC6" gate="A" x="63.5" y="76.2"/>
<instance part="IC6" gate="B" x="81.28" y="86.36"/>
<instance part="IC6" gate="C" x="104.14" y="93.98"/>
<instance part="IC6" gate="D" x="137.16" y="101.6"/>
<instance part="IC7" gate="A" x="101.6" y="116.84" smashed="yes">
<attribute name="NAME" x="104.14" y="120.015" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.568" y="110.49" size="1.778" layer="96"/>
</instance>
<instance part="Q2" gate="1" x="210.82" y="203.2" smashed="yes" rot="MR90">
<attribute name="NAME" x="207.01" y="201.422" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="207.264" y="208.026" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q3" gate="1" x="220.98" y="198.12" smashed="yes" rot="MR90">
<attribute name="NAME" x="217.17" y="196.342" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="217.424" y="202.946" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q4" gate="1" x="231.14" y="193.04" smashed="yes" rot="MR90">
<attribute name="NAME" x="227.33" y="191.262" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="227.584" y="197.866" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q5" gate="1" x="241.3" y="187.96" smashed="yes" rot="MR90">
<attribute name="NAME" x="237.49" y="186.182" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="237.744" y="192.786" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q6" gate="1" x="251.46" y="182.88" smashed="yes" rot="MR90">
<attribute name="NAME" x="247.65" y="181.102" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="247.904" y="187.706" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q7" gate="1" x="261.62" y="177.8" smashed="yes" rot="MR90">
<attribute name="NAME" x="257.81" y="176.022" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="258.064" y="182.626" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q8" gate="1" x="271.78" y="172.72" smashed="yes" rot="MR90">
<attribute name="NAME" x="267.97" y="170.942" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="268.224" y="177.546" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R17" gate="G$1" x="200.66" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="200.406" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="200.152" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R18" gate="G$1" x="210.82" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="210.566" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="210.312" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R19" gate="G$1" x="220.98" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="220.726" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="220.472" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R20" gate="G$1" x="231.14" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="230.886" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="230.632" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R21" gate="G$1" x="241.3" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="241.046" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="240.792" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R22" gate="G$1" x="251.46" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="251.206" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="250.952" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R23" gate="G$1" x="261.62" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="261.366" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="261.112" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R24" gate="G$1" x="271.78" y="99.06" smashed="yes" rot="R90">
<attribute name="NAME" x="271.526" y="103.4034" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="271.018" y="96.012" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R25" gate="G$1" x="266.7" y="149.86" smashed="yes">
<attribute name="NAME" x="263.398" y="151.9174" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="271.272" y="151.892" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R26" gate="G$1" x="269.24" y="144.78" smashed="yes">
<attribute name="NAME" x="265.938" y="146.8374" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="273.812" y="146.812" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R27" gate="G$1" x="271.78" y="139.7" smashed="yes">
<attribute name="NAME" x="268.478" y="141.7574" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="276.352" y="141.732" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R28" gate="G$1" x="274.32" y="134.62" smashed="yes">
<attribute name="NAME" x="271.018" y="136.6774" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="278.892" y="136.652" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R29" gate="G$1" x="276.86" y="129.54" smashed="yes">
<attribute name="NAME" x="273.558" y="131.5974" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="281.432" y="131.572" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R30" gate="G$1" x="281.94" y="124.46" smashed="yes">
<attribute name="NAME" x="278.638" y="126.5174" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="286.512" y="126.492" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R31" gate="G$1" x="284.48" y="119.38" smashed="yes">
<attribute name="NAME" x="281.178" y="121.4374" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="289.052" y="121.412" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="R32" gate="G$1" x="287.02" y="114.3" smashed="yes">
<attribute name="NAME" x="283.718" y="116.3574" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="291.592" y="116.332" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q9" gate="1" x="200.66" y="147.32" smashed="yes" rot="MR90">
<attribute name="NAME" x="203.708" y="149.352" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="199.39" y="152.4" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q10" gate="1" x="210.82" y="142.24" smashed="yes" rot="MR90">
<attribute name="NAME" x="213.868" y="144.272" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="209.804" y="147.32" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q11" gate="1" x="220.98" y="137.16" smashed="yes" rot="MR90">
<attribute name="NAME" x="224.028" y="139.192" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="219.964" y="142.24" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q12" gate="1" x="231.14" y="132.08" smashed="yes" rot="MR90">
<attribute name="NAME" x="234.188" y="134.112" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="230.124" y="137.16" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q13" gate="1" x="241.3" y="127" smashed="yes" rot="MR90">
<attribute name="NAME" x="244.348" y="129.032" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="240.284" y="132.08" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q14" gate="1" x="251.46" y="121.92" smashed="yes" rot="MR90">
<attribute name="NAME" x="254.508" y="123.952" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="250.444" y="127" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q15" gate="1" x="261.62" y="116.84" smashed="yes" rot="MR90">
<attribute name="NAME" x="264.668" y="118.872" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="260.604" y="121.92" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q16" gate="1" x="271.78" y="111.76" smashed="yes" rot="MR90">
<attribute name="NAME" x="274.828" y="113.792" size="1.778" layer="95" rot="MR180"/>
<attribute name="VALUE" x="270.764" y="116.84" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="Q17" gate="1" x="167.64" y="71.12" smashed="yes">
<attribute name="NAME" x="175.514" y="72.898" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="Q18" gate="1" x="274.32" y="81.28" smashed="yes" rot="MR180">
<attribute name="NAME" x="274.32" y="85.09" size="1.778" layer="95" rot="MR0"/>
<attribute name="VALUE" x="278.13" y="80.01" size="1.778" layer="96"/>
</instance>
<instance part="R54" gate="G$1" x="160.02" y="66.04" smashed="yes" rot="R270">
<attribute name="NAME" x="154.178" y="62.9666" size="1.778" layer="95"/>
<attribute name="VALUE" x="154.178" y="66.04" size="1.778" layer="96"/>
</instance>
<instance part="R55" gate="G$1" x="220.98" y="88.9" smashed="yes" rot="R90">
<attribute name="NAME" x="218.948" y="89.4334" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="226.822" y="91.44" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R56" gate="G$1" x="177.8" y="78.74" smashed="yes">
<attribute name="NAME" x="176.784" y="82.3214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="180.34" y="76.962" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SG1" gate="G$1" x="187.96" y="81.28" smashed="yes">
<attribute name="NAME" x="180.34" y="82.55" size="1.778" layer="95"/>
</instance>
<instance part="P+2" gate="1" x="200.66" y="71.12" smashed="yes">
<attribute name="VALUE" x="202.438" y="74.93" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IC7" gate="C" x="256.54" y="81.28"/>
<instance part="IC7" gate="B" x="233.68" y="81.28"/>
<instance part="IC7" gate="D" x="309.88" y="93.98"/>
<instance part="IC7" gate="E" x="309.88" y="81.28"/>
<instance part="IC7" gate="F" x="309.88" y="68.58"/>
<instance part="R57" gate="G$1" x="287.02" y="93.98" smashed="yes" rot="R180">
<attribute name="NAME" x="287.02" y="95.7326" size="1.778" layer="95"/>
<attribute name="VALUE" x="285.496" y="90.424" size="1.778" layer="96"/>
</instance>
<instance part="GND2" gate="1" x="54.864" y="170.18"/>
<instance part="J9" gate="-1" x="309.88" y="106.68" smashed="yes">
<attribute name="NAME" x="312.42" y="105.918" size="1.524" layer="95"/>
<attribute name="VALUE" x="309.118" y="102.997" size="1.778" layer="96"/>
</instance>
<instance part="J9" gate="-2" x="309.88" y="114.3"/>
<instance part="J9" gate="-3" x="309.88" y="119.38"/>
<instance part="J9" gate="-4" x="309.88" y="124.46"/>
<instance part="J9" gate="-5" x="309.88" y="129.54"/>
<instance part="J9" gate="-6" x="309.88" y="134.62"/>
<instance part="J9" gate="-7" x="309.88" y="139.7"/>
<instance part="J9" gate="-8" x="309.88" y="144.78"/>
<instance part="J9" gate="-9" x="309.88" y="149.86"/>
<instance part="J9" gate="-10" x="309.88" y="175.26"/>
<instance part="J9" gate="-11" x="309.88" y="180.34"/>
<instance part="P+3" gate="1" x="292.1" y="111.76" smashed="yes">
<attribute name="VALUE" x="290.576" y="110.49" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="P+4" gate="1" x="307.34" y="220.98" smashed="yes">
<attribute name="VALUE" x="305.054" y="220.98" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R58" gate="G$1" x="274.32" y="68.58" smashed="yes">
<attribute name="NAME" x="276.606" y="66.5734" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="273.05" y="71.882" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C12" gate="G$1" x="259.08" y="66.04" smashed="yes">
<attribute name="NAME" x="251.46" y="66.04" size="1.778" layer="95"/>
<attribute name="VALUE" x="252.73" y="60.452" size="1.778" layer="96"/>
</instance>
<instance part="R59" gate="G$1" x="266.7" y="63.5" smashed="yes" rot="R90">
<attribute name="NAME" x="260.604" y="60.6806" size="1.778" layer="95"/>
<attribute name="VALUE" x="268.732" y="61.468" size="1.778" layer="96"/>
</instance>
<instance part="GND3" gate="1" x="266.7" y="55.88"/>
<instance part="P6" gate="-3" x="281.94" y="73.66"/>
<instance part="P6" gate="-4" x="281.94" y="68.58"/>
<instance part="J8" gate="-1" x="309.88" y="213.36" smashed="yes">
<attribute name="NAME" x="312.42" y="212.598" size="1.524" layer="95"/>
<attribute name="VALUE" x="32.258" y="199.517" size="1.778" layer="96"/>
</instance>
<instance part="J8" gate="-2" x="38.1" y="195.58" rot="MR0"/>
<instance part="J8" gate="-3" x="43.18" y="129.54" rot="MR0"/>
<instance part="J8" gate="-4" x="43.18" y="132.08" rot="MR0"/>
<instance part="J8" gate="-5" x="43.18" y="134.62" rot="MR0"/>
<instance part="J8" gate="-6" x="43.18" y="137.16" rot="MR0"/>
<instance part="J8" gate="-7" x="43.18" y="139.7" rot="MR0"/>
<instance part="J8" gate="-8" x="43.18" y="142.24" rot="MR0"/>
<instance part="J8" gate="-9" x="43.18" y="144.78" rot="MR0"/>
<instance part="J8" gate="-10" x="43.18" y="147.32" rot="MR0"/>
<instance part="J8" gate="-11" x="40.64" y="175.26" rot="MR0"/>
<instance part="J8" gate="-12" x="40.64" y="177.8" rot="MR0"/>
<instance part="J8" gate="-13" x="40.64" y="180.34" rot="MR0"/>
<instance part="J8" gate="-14" x="40.64" y="182.88" rot="MR0"/>
<instance part="J8" gate="-15" x="40.64" y="185.42" rot="MR0"/>
<instance part="J8" gate="-16" x="40.64" y="187.96" rot="MR0"/>
<instance part="J8" gate="-17" x="309.88" y="205.74"/>
<instance part="J8" gate="-18" x="309.88" y="210.82"/>
<instance part="GND1" gate="1" x="190.5" y="137.16"/>
<instance part="GND4" gate="1" x="119.38" y="113.03"/>
<instance part="GND5" gate="1" x="170.18" y="58.42"/>
<instance part="R60" gate="G$1" x="137.16" y="208.28" smashed="yes" rot="R180">
<attribute name="NAME" x="132.3086" y="204.978" size="1.778" layer="95"/>
<attribute name="VALUE" x="139.954" y="209.55" size="1.778" layer="96"/>
</instance>
<instance part="R61" gate="G$1" x="302.26" y="106.68" smashed="yes">
<attribute name="NAME" x="298.958" y="108.7374" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="304.292" y="108.712" size="1.778" layer="96" rot="MR180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$3" class="0">
<segment>
<wire x1="210.82" y1="182.88" x2="210.82" y2="198.12" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QB"/>
<wire x1="193.04" y1="182.88" x2="210.82" y2="182.88" width="0.1524" layer="91"/>
<pinref part="Q2" gate="1" pin="G"/>
<pinref part="R2" gate="G$1" pin="2"/>
<wire x1="210.82" y1="165.1" x2="210.82" y2="182.88" width="0.1524" layer="91"/>
<junction x="210.82" y="182.88"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<wire x1="220.98" y1="193.04" x2="220.98" y2="180.34" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QC"/>
<wire x1="193.04" y1="180.34" x2="220.98" y2="180.34" width="0.1524" layer="91"/>
<pinref part="Q3" gate="1" pin="G"/>
<pinref part="R3" gate="G$1" pin="2"/>
<wire x1="220.98" y1="165.1" x2="220.98" y2="180.34" width="0.1524" layer="91"/>
<junction x="220.98" y="180.34"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<wire x1="231.14" y1="187.96" x2="231.14" y2="177.8" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QD"/>
<wire x1="193.04" y1="177.8" x2="231.14" y2="177.8" width="0.1524" layer="91"/>
<pinref part="Q4" gate="1" pin="G"/>
<pinref part="R4" gate="G$1" pin="2"/>
<wire x1="231.14" y1="165.1" x2="231.14" y2="177.8" width="0.1524" layer="91"/>
<junction x="231.14" y="177.8"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<wire x1="241.3" y1="182.88" x2="241.3" y2="175.26" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QE"/>
<wire x1="193.04" y1="175.26" x2="241.3" y2="175.26" width="0.1524" layer="91"/>
<pinref part="Q5" gate="1" pin="G"/>
<pinref part="R5" gate="G$1" pin="2"/>
<wire x1="241.3" y1="165.1" x2="241.3" y2="175.26" width="0.1524" layer="91"/>
<junction x="241.3" y="175.26"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<wire x1="261.62" y1="172.72" x2="261.62" y2="170.18" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QG"/>
<wire x1="193.04" y1="170.18" x2="261.62" y2="170.18" width="0.1524" layer="91"/>
<pinref part="Q7" gate="1" pin="G"/>
<pinref part="R7" gate="G$1" pin="2"/>
<wire x1="261.62" y1="165.1" x2="261.62" y2="170.18" width="0.1524" layer="91"/>
<junction x="261.62" y="170.18"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="IC4" gate="A" pin="QH"/>
<wire x1="193.04" y1="167.64" x2="271.78" y2="167.64" width="0.1524" layer="91"/>
<pinref part="Q8" gate="1" pin="G"/>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="271.78" y1="165.1" x2="271.78" y2="167.64" width="0.1524" layer="91"/>
<junction x="271.78" y="167.64"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="Q1" gate="1" pin="D"/>
<pinref part="R9" gate="G$1" pin="1"/>
<wire x1="205.74" y1="210.82" x2="281.94" y2="210.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<wire x1="215.9" y1="205.74" x2="281.94" y2="205.74" width="0.1524" layer="91"/>
<pinref part="R10" gate="G$1" pin="1"/>
<pinref part="Q2" gate="1" pin="D"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<wire x1="226.06" y1="200.66" x2="266.7" y2="200.66" width="0.1524" layer="91"/>
<pinref part="R11" gate="G$1" pin="1"/>
<pinref part="Q3" gate="1" pin="D"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<wire x1="236.22" y1="195.58" x2="269.24" y2="195.58" width="0.1524" layer="91"/>
<pinref part="R12" gate="G$1" pin="1"/>
<pinref part="Q4" gate="1" pin="D"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<wire x1="246.38" y1="190.5" x2="271.78" y2="190.5" width="0.1524" layer="91"/>
<pinref part="R13" gate="G$1" pin="1"/>
<pinref part="Q5" gate="1" pin="D"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<wire x1="256.54" y1="185.42" x2="274.32" y2="185.42" width="0.1524" layer="91"/>
<pinref part="R14" gate="G$1" pin="1"/>
<pinref part="Q6" gate="1" pin="D"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<wire x1="266.7" y1="180.34" x2="279.4" y2="180.34" width="0.1524" layer="91"/>
<pinref part="R15" gate="G$1" pin="1"/>
<pinref part="Q7" gate="1" pin="D"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<wire x1="276.86" y1="175.26" x2="281.94" y2="175.26" width="0.1524" layer="91"/>
<pinref part="R16" gate="G$1" pin="1"/>
<pinref part="Q8" gate="1" pin="D"/>
</segment>
</net>
<net name="VCC1" class="0">
</net>
<net name="N$18" class="0">
<segment>
<pinref part="IC4" gate="A" pin="QH*"/>
<wire x1="193.04" y1="162.56" x2="193.04" y2="152.4" width="0.1524" layer="91"/>
<wire x1="193.04" y1="152.4" x2="167.64" y2="152.4" width="0.1524" layer="91"/>
<wire x1="167.64" y1="152.4" x2="167.64" y2="124.46" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="SER"/>
</segment>
</net>
<net name="N$35" class="0">
<segment>
<pinref part="IC2" gate="A" pin="QH"/>
<wire x1="149.86" y1="193.04" x2="152.4" y2="193.04" width="0.1524" layer="91"/>
<wire x1="152.4" y1="193.04" x2="152.4" y2="157.48" width="0.1524" layer="91"/>
<wire x1="152.4" y1="157.48" x2="124.46" y2="157.48" width="0.1524" layer="91"/>
<wire x1="124.46" y1="157.48" x2="124.46" y2="149.86" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="SER"/>
<wire x1="124.46" y1="149.86" x2="127" y2="149.86" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$56" class="0">
<segment>
<wire x1="111.76" y1="93.98" x2="114.3" y2="93.98" width="0.1524" layer="91"/>
<wire x1="114.3" y1="93.98" x2="157.48" y2="93.98" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="SCK"/>
<wire x1="157.48" y1="93.98" x2="157.48" y2="119.38" width="0.1524" layer="91"/>
<wire x1="157.48" y1="119.38" x2="157.48" y2="180.34" width="0.1524" layer="91"/>
<wire x1="157.48" y1="180.34" x2="167.64" y2="180.34" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="SCK"/>
<wire x1="167.64" y1="119.38" x2="157.48" y2="119.38" width="0.1524" layer="91"/>
<wire x1="114.3" y1="93.98" x2="114.3" y2="124.46" width="0.1524" layer="91"/>
<wire x1="114.3" y1="124.46" x2="114.3" y2="170.18" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="CLK"/>
<wire x1="124.46" y1="170.18" x2="114.3" y2="170.18" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="CLK"/>
<wire x1="124.46" y1="124.46" x2="114.3" y2="124.46" width="0.1524" layer="91"/>
<junction x="114.3" y="124.46"/>
<junction x="114.3" y="93.98"/>
<junction x="157.48" y="119.38"/>
<pinref part="IC6" gate="C" pin="O"/>
</segment>
</net>
<net name="OUT_15" class="0">
<segment>
<pinref part="R9" gate="G$1" pin="2"/>
<wire x1="292.1" y1="210.82" x2="307.34" y2="210.82" width="0.1524" layer="91"/>
<label x="298.704" y="211.582" size="1.778" layer="95"/>
<pinref part="J8" gate="-18" pin="S"/>
</segment>
</net>
<net name="OUT_14" class="0">
<segment>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="292.1" y1="205.74" x2="307.34" y2="205.74" width="0.1524" layer="91"/>
<label x="298.958" y="206.502" size="1.778" layer="95"/>
<pinref part="J8" gate="-17" pin="S"/>
</segment>
</net>
<net name="OUT_13" class="0">
<segment>
<pinref part="R11" gate="G$1" pin="2"/>
<wire x1="276.86" y1="200.66" x2="294.64" y2="200.66" width="0.1524" layer="91"/>
<label x="291.338" y="201.422" size="1.778" layer="95"/>
</segment>
</net>
<net name="OUT_12" class="0">
<segment>
<pinref part="R12" gate="G$1" pin="2"/>
<wire x1="279.4" y1="195.58" x2="294.64" y2="195.58" width="0.1524" layer="91"/>
<label x="291.084" y="196.342" size="1.778" layer="95"/>
</segment>
</net>
<net name="OUT_11" class="0">
<segment>
<pinref part="R13" gate="G$1" pin="2"/>
<wire x1="281.94" y1="190.5" x2="294.64" y2="190.5" width="0.1524" layer="91"/>
<label x="291.084" y="191.262" size="1.778" layer="95"/>
</segment>
</net>
<net name="OUT_10" class="0">
<segment>
<pinref part="R14" gate="G$1" pin="2"/>
<wire x1="284.48" y1="185.42" x2="294.64" y2="185.42" width="0.1524" layer="91"/>
<label x="291.084" y="186.182" size="1.778" layer="95"/>
</segment>
</net>
<net name="OUT_9" class="0">
<segment>
<pinref part="R15" gate="G$1" pin="2"/>
<wire x1="289.56" y1="180.34" x2="307.34" y2="180.34" width="0.1524" layer="91"/>
<label x="298.704" y="181.102" size="1.778" layer="95"/>
<pinref part="J9" gate="-11" pin="S"/>
</segment>
</net>
<net name="OUT_8" class="0">
<segment>
<pinref part="R16" gate="G$1" pin="2"/>
<wire x1="292.1" y1="175.26" x2="307.34" y2="175.26" width="0.1524" layer="91"/>
<label x="298.704" y="176.022" size="1.778" layer="95"/>
<pinref part="J9" gate="-10" pin="S"/>
</segment>
</net>
<net name="OUT_1" class="0">
<segment>
<pinref part="R31" gate="G$1" pin="2"/>
<wire x1="289.56" y1="119.38" x2="307.34" y2="119.38" width="0.1524" layer="91"/>
<label x="298.704" y="120.142" size="1.778" layer="95"/>
<pinref part="J9" gate="-3" pin="S"/>
</segment>
</net>
<net name="OUT_0" class="0">
<segment>
<pinref part="R32" gate="G$1" pin="2"/>
<wire x1="292.1" y1="114.3" x2="307.34" y2="114.3" width="0.1524" layer="91"/>
<label x="298.45" y="115.062" size="1.778" layer="95"/>
<pinref part="J9" gate="-2" pin="S"/>
</segment>
</net>
<net name="N$57" class="0">
<segment>
<pinref part="IC4" gate="A" pin="SER"/>
<wire x1="167.64" y1="185.42" x2="154.94" y2="185.42" width="0.1524" layer="91"/>
<wire x1="154.94" y1="185.42" x2="154.94" y2="101.6" width="0.1524" layer="91"/>
<wire x1="154.94" y1="101.6" x2="144.78" y2="101.6" width="0.1524" layer="91"/>
<pinref part="IC6" gate="D" pin="O"/>
</segment>
</net>
<net name="N$58" class="0">
<segment>
<pinref part="IC4" gate="A" pin="SCL"/>
<wire x1="167.64" y1="177.8" x2="165.1" y2="177.8" width="0.1524" layer="91"/>
<wire x1="165.1" y1="177.8" x2="165.1" y2="116.84" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="SCL"/>
<wire x1="165.1" y1="116.84" x2="167.64" y2="116.84" width="0.1524" layer="91"/>
<pinref part="R33" gate="G$1" pin="1"/>
<wire x1="165.1" y1="193.04" x2="165.1" y2="177.8" width="0.1524" layer="91"/>
<junction x="165.1" y="177.8"/>
</segment>
</net>
<net name="N$59" class="0">
<segment>
<pinref part="IC4" gate="A" pin="RCK"/>
<wire x1="167.64" y1="172.72" x2="160.02" y2="172.72" width="0.1524" layer="91"/>
<wire x1="160.02" y1="172.72" x2="160.02" y2="111.76" width="0.1524" layer="91"/>
<wire x1="160.02" y1="111.76" x2="160.02" y2="86.36" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="RCK"/>
<wire x1="167.64" y1="111.76" x2="160.02" y2="111.76" width="0.1524" layer="91"/>
<junction x="160.02" y="111.76"/>
<wire x1="160.02" y1="86.36" x2="91.44" y2="86.36" width="0.1524" layer="91"/>
<wire x1="91.44" y1="86.36" x2="88.9" y2="86.36" width="0.1524" layer="91"/>
<wire x1="91.44" y1="116.84" x2="91.44" y2="86.36" width="0.1524" layer="91"/>
<junction x="91.44" y="86.36"/>
<pinref part="IC6" gate="B" pin="O"/>
<pinref part="IC7" gate="A" pin="I"/>
</segment>
</net>
<net name="N$60" class="0">
<segment>
<pinref part="IC4" gate="A" pin="G"/>
<wire x1="167.64" y1="162.56" x2="162.56" y2="162.56" width="0.1524" layer="91"/>
<wire x1="162.56" y1="162.56" x2="162.56" y2="101.6" width="0.1524" layer="91"/>
<wire x1="162.56" y1="101.6" x2="162.56" y2="76.2" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="G"/>
<wire x1="167.64" y1="101.6" x2="162.56" y2="101.6" width="0.1524" layer="91"/>
<junction x="162.56" y="101.6"/>
<wire x1="162.56" y1="76.2" x2="71.12" y2="76.2" width="0.1524" layer="91"/>
<pinref part="IC6" gate="A" pin="O"/>
</segment>
</net>
<net name="IN_15" class="0">
<segment>
<pinref part="R34" gate="G$1" pin="1"/>
<wire x1="104.14" y1="195.58" x2="104.14" y2="193.04" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="A"/>
<wire x1="124.46" y1="193.04" x2="104.14" y2="193.04" width="0.1524" layer="91"/>
<label x="44.196" y="193.294" size="1.778" layer="95"/>
<wire x1="104.14" y1="193.04" x2="43.18" y2="193.04" width="0.1524" layer="91"/>
<junction x="104.14" y="193.04"/>
</segment>
</net>
<net name="IN_14" class="0">
<segment>
<pinref part="R35" gate="G$1" pin="1"/>
<wire x1="96.52" y1="195.58" x2="96.52" y2="190.5" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="B"/>
<wire x1="124.46" y1="190.5" x2="96.52" y2="190.5" width="0.1524" layer="91"/>
<label x="44.196" y="191.008" size="1.778" layer="95"/>
<wire x1="96.52" y1="190.5" x2="43.18" y2="190.5" width="0.1524" layer="91"/>
<junction x="96.52" y="190.5"/>
</segment>
</net>
<net name="IN_13" class="0">
<segment>
<pinref part="R36" gate="G$1" pin="1"/>
<wire x1="88.9" y1="195.58" x2="88.9" y2="187.96" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="C"/>
<wire x1="124.46" y1="187.96" x2="88.9" y2="187.96" width="0.1524" layer="91"/>
<label x="44.196" y="188.468" size="1.778" layer="95"/>
<wire x1="45.72" y1="187.96" x2="88.9" y2="187.96" width="0.1524" layer="91"/>
<junction x="88.9" y="187.96"/>
</segment>
</net>
<net name="IN_12" class="0">
<segment>
<pinref part="R37" gate="G$1" pin="1"/>
<wire x1="81.28" y1="195.58" x2="81.28" y2="185.42" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="D"/>
<wire x1="124.46" y1="185.42" x2="81.28" y2="185.42" width="0.1524" layer="91"/>
<label x="44.196" y="185.928" size="1.778" layer="95"/>
<pinref part="J8" gate="-15" pin="S"/>
<wire x1="81.28" y1="185.42" x2="43.18" y2="185.42" width="0.1524" layer="91"/>
<junction x="81.28" y="185.42"/>
</segment>
</net>
<net name="IN_11" class="0">
<segment>
<pinref part="R38" gate="G$1" pin="1"/>
<wire x1="73.66" y1="195.58" x2="73.66" y2="182.88" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="E"/>
<wire x1="124.46" y1="182.88" x2="73.66" y2="182.88" width="0.1524" layer="91"/>
<label x="44.196" y="183.388" size="1.778" layer="95"/>
<pinref part="J8" gate="-14" pin="S"/>
<wire x1="43.18" y1="182.88" x2="73.66" y2="182.88" width="0.1524" layer="91"/>
<junction x="73.66" y="182.88"/>
</segment>
</net>
<net name="IN_10" class="0">
<segment>
<pinref part="R39" gate="G$1" pin="1"/>
<wire x1="66.04" y1="195.58" x2="66.04" y2="180.34" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="F"/>
<wire x1="124.46" y1="180.34" x2="66.04" y2="180.34" width="0.1524" layer="91"/>
<label x="44.196" y="180.848" size="1.778" layer="95"/>
<pinref part="J8" gate="-13" pin="S"/>
<wire x1="66.04" y1="180.34" x2="43.18" y2="180.34" width="0.1524" layer="91"/>
<junction x="66.04" y="180.34"/>
</segment>
</net>
<net name="IN_9" class="0">
<segment>
<pinref part="R40" gate="G$1" pin="1"/>
<wire x1="58.42" y1="177.8" x2="58.42" y2="195.58" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="G"/>
<wire x1="124.46" y1="177.8" x2="58.42" y2="177.8" width="0.1524" layer="91"/>
<label x="44.196" y="178.308" size="1.778" layer="95"/>
<pinref part="J8" gate="-12" pin="S"/>
<wire x1="43.18" y1="177.8" x2="58.42" y2="177.8" width="0.1524" layer="91"/>
<junction x="58.42" y="177.8"/>
</segment>
</net>
<net name="IN_8" class="0">
<segment>
<pinref part="R41" gate="G$1" pin="1"/>
<wire x1="50.8" y1="195.58" x2="50.8" y2="175.26" width="0.1524" layer="91"/>
<label x="44.196" y="175.768" size="1.778" layer="95"/>
<pinref part="IC2" gate="A" pin="H"/>
<wire x1="124.46" y1="175.26" x2="50.8" y2="175.26" width="0.1524" layer="91"/>
<pinref part="J8" gate="-11" pin="S"/>
<wire x1="50.8" y1="175.26" x2="43.18" y2="175.26" width="0.1524" layer="91"/>
<junction x="50.8" y="175.26"/>
</segment>
</net>
<net name="IN_7" class="0">
<segment>
<pinref part="R42" gate="G$1" pin="1"/>
<wire x1="104.14" y1="149.86" x2="104.14" y2="147.32" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="A"/>
<wire x1="124.46" y1="147.32" x2="104.14" y2="147.32" width="0.1524" layer="91"/>
<label x="42.926" y="147.828" size="1.778" layer="95"/>
<pinref part="J8" gate="-10" pin="S"/>
<wire x1="45.72" y1="147.32" x2="104.14" y2="147.32" width="0.1524" layer="91"/>
<junction x="104.14" y="147.32"/>
</segment>
</net>
<net name="IN_6" class="0">
<segment>
<pinref part="R43" gate="G$1" pin="1"/>
<wire x1="96.52" y1="144.78" x2="96.52" y2="149.86" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="B"/>
<wire x1="124.46" y1="144.78" x2="96.52" y2="144.78" width="0.1524" layer="91"/>
<label x="42.926" y="145.288" size="1.778" layer="95"/>
<pinref part="J8" gate="-9" pin="S"/>
<wire x1="96.52" y1="144.78" x2="45.72" y2="144.78" width="0.1524" layer="91"/>
<junction x="96.52" y="144.78"/>
</segment>
</net>
<net name="IN_5" class="0">
<segment>
<pinref part="R44" gate="G$1" pin="1"/>
<wire x1="88.9" y1="149.86" x2="88.9" y2="142.24" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="C"/>
<wire x1="124.46" y1="142.24" x2="88.9" y2="142.24" width="0.1524" layer="91"/>
<label x="42.926" y="142.748" size="1.778" layer="95"/>
<pinref part="J8" gate="-8" pin="S"/>
<wire x1="45.72" y1="142.24" x2="88.9" y2="142.24" width="0.1524" layer="91"/>
<junction x="88.9" y="142.24"/>
</segment>
</net>
<net name="IN_4" class="0">
<segment>
<pinref part="R45" gate="G$1" pin="1"/>
<wire x1="81.28" y1="139.7" x2="81.28" y2="149.86" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="D"/>
<wire x1="124.46" y1="139.7" x2="81.28" y2="139.7" width="0.1524" layer="91"/>
<label x="42.926" y="140.208" size="1.778" layer="95"/>
<pinref part="J8" gate="-7" pin="S"/>
<wire x1="81.28" y1="139.7" x2="45.72" y2="139.7" width="0.1524" layer="91"/>
<junction x="81.28" y="139.7"/>
</segment>
</net>
<net name="IN_3" class="0">
<segment>
<pinref part="R46" gate="G$1" pin="1"/>
<wire x1="73.66" y1="149.86" x2="73.66" y2="137.16" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="E"/>
<wire x1="124.46" y1="137.16" x2="73.66" y2="137.16" width="0.1524" layer="91"/>
<label x="42.926" y="137.668" size="1.778" layer="95"/>
<pinref part="J8" gate="-6" pin="S"/>
<wire x1="45.72" y1="137.16" x2="73.66" y2="137.16" width="0.1524" layer="91"/>
<junction x="73.66" y="137.16"/>
</segment>
</net>
<net name="IN_2" class="0">
<segment>
<pinref part="R47" gate="G$1" pin="1"/>
<wire x1="66.04" y1="134.62" x2="66.04" y2="149.86" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="F"/>
<wire x1="124.46" y1="134.62" x2="66.04" y2="134.62" width="0.1524" layer="91"/>
<label x="43.18" y="135.128" size="1.778" layer="95"/>
<pinref part="J8" gate="-5" pin="S"/>
<wire x1="66.04" y1="134.62" x2="45.72" y2="134.62" width="0.1524" layer="91"/>
<junction x="66.04" y="134.62"/>
</segment>
</net>
<net name="IN_1" class="0">
<segment>
<pinref part="R48" gate="G$1" pin="1"/>
<wire x1="58.42" y1="149.86" x2="58.42" y2="132.08" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="G"/>
<wire x1="124.46" y1="132.08" x2="58.42" y2="132.08" width="0.1524" layer="91"/>
<label x="42.926" y="132.588" size="1.778" layer="95"/>
<pinref part="J8" gate="-4" pin="S"/>
<wire x1="45.72" y1="132.08" x2="58.42" y2="132.08" width="0.1524" layer="91"/>
<junction x="58.42" y="132.08"/>
</segment>
</net>
<net name="IN_0" class="0">
<segment>
<pinref part="IC3" gate="A" pin="H"/>
<wire x1="124.46" y1="129.54" x2="50.8" y2="129.54" width="0.1524" layer="91"/>
<label x="42.926" y="130.048" size="1.778" layer="95"/>
<pinref part="R49" gate="G$1" pin="1"/>
<wire x1="50.8" y1="149.86" x2="50.8" y2="129.54" width="0.1524" layer="91"/>
<pinref part="J8" gate="-3" pin="S"/>
<wire x1="50.8" y1="129.54" x2="45.72" y2="129.54" width="0.1524" layer="91"/>
<junction x="50.8" y="129.54"/>
</segment>
<segment>
<pinref part="R58" gate="G$1" pin="1"/>
<wire x1="269.24" y1="68.58" x2="266.7" y2="68.58" width="0.1524" layer="91"/>
<label x="244.602" y="67.564" size="1.778" layer="95"/>
<pinref part="R59" gate="G$1" pin="2"/>
<wire x1="266.7" y1="68.58" x2="259.08" y2="68.58" width="0.1524" layer="91"/>
<junction x="266.7" y="68.58"/>
<pinref part="C12" gate="G$1" pin="1"/>
<wire x1="259.08" y1="68.58" x2="251.46" y2="68.58" width="0.1524" layer="91"/>
<junction x="259.08" y="68.58"/>
</segment>
</net>
<net name="MISO" class="0">
<segment>
<pinref part="IC3" gate="A" pin="QH"/>
<wire x1="149.86" y1="147.32" x2="152.4" y2="147.32" width="0.1524" layer="91"/>
<wire x1="152.4" y1="147.32" x2="152.4" y2="109.22" width="0.1524" layer="91"/>
<wire x1="152.4" y1="109.22" x2="40.64" y2="109.22" width="0.1524" layer="91"/>
<label x="40.386" y="108.204" size="1.778" layer="95" rot="MR0"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="P+1" gate="1" pin="+5V"/>
<wire x1="165.1" y1="203.2" x2="165.1" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R33" gate="G$1" pin="2"/>
<pinref part="R60" gate="G$1" pin="1"/>
<wire x1="165.1" y1="208.28" x2="165.1" y2="218.44" width="0.1524" layer="91"/>
<wire x1="142.24" y1="208.28" x2="165.1" y2="208.28" width="0.1524" layer="91"/>
<junction x="165.1" y="208.28"/>
</segment>
</net>
<net name="SCLK" class="0">
<segment>
<wire x1="96.52" y1="96.52" x2="96.52" y2="93.98" width="0.1524" layer="91"/>
<wire x1="96.52" y1="93.98" x2="96.52" y2="91.44" width="0.1524" layer="91"/>
<wire x1="96.52" y1="93.98" x2="71.12" y2="93.98" width="0.1524" layer="91"/>
<junction x="96.52" y="93.98"/>
<pinref part="R51" gate="G$1" pin="1"/>
<wire x1="71.12" y1="93.98" x2="40.64" y2="93.98" width="0.1524" layer="91"/>
<wire x1="71.12" y1="111.76" x2="71.12" y2="93.98" width="0.1524" layer="91"/>
<junction x="71.12" y="93.98"/>
<pinref part="IC6" gate="C" pin="I0"/>
<pinref part="IC6" gate="C" pin="I1"/>
<label x="40.386" y="92.964" size="1.778" layer="95" rot="MR0"/>
</segment>
</net>
<net name="NSS" class="0">
<segment>
<wire x1="73.66" y1="88.9" x2="73.66" y2="86.36" width="0.1524" layer="91"/>
<wire x1="73.66" y1="86.36" x2="73.66" y2="83.82" width="0.1524" layer="91"/>
<wire x1="73.66" y1="86.36" x2="60.96" y2="86.36" width="0.1524" layer="91"/>
<junction x="73.66" y="86.36"/>
<pinref part="R52" gate="G$1" pin="1"/>
<wire x1="60.96" y1="86.36" x2="40.64" y2="86.36" width="0.1524" layer="91"/>
<wire x1="60.96" y1="111.76" x2="60.96" y2="86.36" width="0.1524" layer="91"/>
<junction x="60.96" y="86.36"/>
<pinref part="IC6" gate="B" pin="I0"/>
<pinref part="IC6" gate="B" pin="I1"/>
<label x="40.64" y="85.344" size="1.778" layer="95" rot="MR0"/>
</segment>
</net>
<net name="LED_PWM" class="0">
<segment>
<wire x1="55.88" y1="78.74" x2="55.88" y2="76.2" width="0.1524" layer="91"/>
<wire x1="55.88" y1="76.2" x2="55.88" y2="73.66" width="0.1524" layer="91"/>
<wire x1="55.88" y1="76.2" x2="50.8" y2="76.2" width="0.1524" layer="91"/>
<junction x="55.88" y="76.2"/>
<pinref part="R53" gate="G$1" pin="1"/>
<wire x1="50.8" y1="111.76" x2="50.8" y2="76.2" width="0.1524" layer="91"/>
<wire x1="50.8" y1="76.2" x2="40.64" y2="76.2" width="0.1524" layer="91"/>
<junction x="50.8" y="76.2"/>
<pinref part="IC6" gate="A" pin="I0"/>
<pinref part="IC6" gate="A" pin="I1"/>
<label x="40.132" y="75.184" size="1.778" layer="95" rot="MR0"/>
</segment>
</net>
<net name="MOSI" class="0">
<segment>
<wire x1="129.54" y1="99.06" x2="129.54" y2="101.6" width="0.1524" layer="91"/>
<wire x1="129.54" y1="101.6" x2="129.54" y2="104.14" width="0.1524" layer="91"/>
<wire x1="129.54" y1="101.6" x2="78.74" y2="101.6" width="0.1524" layer="91"/>
<junction x="129.54" y="101.6"/>
<pinref part="R50" gate="G$1" pin="1"/>
<wire x1="78.74" y1="101.6" x2="40.64" y2="101.6" width="0.1524" layer="91"/>
<wire x1="78.74" y1="111.76" x2="78.74" y2="101.6" width="0.1524" layer="91"/>
<junction x="78.74" y="101.6"/>
<pinref part="IC6" gate="D" pin="I0"/>
<pinref part="IC6" gate="D" pin="I1"/>
<label x="40.64" y="100.838" size="1.778" layer="95" rot="MR0"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<wire x1="251.46" y1="177.8" x2="251.46" y2="172.72" width="0.1524" layer="91"/>
<pinref part="IC4" gate="A" pin="QF"/>
<wire x1="193.04" y1="172.72" x2="251.46" y2="172.72" width="0.1524" layer="91"/>
<pinref part="Q6" gate="1" pin="G"/>
<pinref part="R6" gate="G$1" pin="2"/>
<wire x1="251.46" y1="165.1" x2="251.46" y2="172.72" width="0.1524" layer="91"/>
<junction x="251.46" y="172.72"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="IC4" gate="A" pin="QA"/>
<wire x1="193.04" y1="185.42" x2="200.66" y2="185.42" width="0.1524" layer="91"/>
<pinref part="Q1" gate="1" pin="G"/>
<wire x1="200.66" y1="185.42" x2="200.66" y2="203.2" width="0.1524" layer="91"/>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="200.66" y1="165.1" x2="200.66" y2="185.42" width="0.1524" layer="91"/>
<junction x="200.66" y="185.42"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<wire x1="210.82" y1="121.92" x2="210.82" y2="137.16" width="0.1524" layer="91"/>
<wire x1="193.04" y1="121.92" x2="210.82" y2="121.92" width="0.1524" layer="91"/>
<pinref part="R18" gate="G$1" pin="2"/>
<wire x1="210.82" y1="104.14" x2="210.82" y2="121.92" width="0.1524" layer="91"/>
<junction x="210.82" y="121.92"/>
<pinref part="IC5" gate="A" pin="QB"/>
<pinref part="Q10" gate="1" pin="G"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<wire x1="220.98" y1="132.08" x2="220.98" y2="119.38" width="0.1524" layer="91"/>
<wire x1="193.04" y1="119.38" x2="220.98" y2="119.38" width="0.1524" layer="91"/>
<pinref part="R19" gate="G$1" pin="2"/>
<wire x1="220.98" y1="104.14" x2="220.98" y2="119.38" width="0.1524" layer="91"/>
<junction x="220.98" y="119.38"/>
<pinref part="IC5" gate="A" pin="QC"/>
<pinref part="Q11" gate="1" pin="G"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<wire x1="231.14" y1="127" x2="231.14" y2="116.84" width="0.1524" layer="91"/>
<wire x1="193.04" y1="116.84" x2="231.14" y2="116.84" width="0.1524" layer="91"/>
<pinref part="R20" gate="G$1" pin="2"/>
<wire x1="231.14" y1="104.14" x2="231.14" y2="116.84" width="0.1524" layer="91"/>
<junction x="231.14" y="116.84"/>
<pinref part="IC5" gate="A" pin="QD"/>
<pinref part="Q12" gate="1" pin="G"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<wire x1="241.3" y1="121.92" x2="241.3" y2="114.3" width="0.1524" layer="91"/>
<wire x1="193.04" y1="114.3" x2="241.3" y2="114.3" width="0.1524" layer="91"/>
<pinref part="R21" gate="G$1" pin="2"/>
<wire x1="241.3" y1="104.14" x2="241.3" y2="114.3" width="0.1524" layer="91"/>
<junction x="241.3" y="114.3"/>
<pinref part="IC5" gate="A" pin="QE"/>
<pinref part="Q13" gate="1" pin="G"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<wire x1="261.62" y1="111.76" x2="261.62" y2="109.22" width="0.1524" layer="91"/>
<wire x1="193.04" y1="109.22" x2="261.62" y2="109.22" width="0.1524" layer="91"/>
<pinref part="R23" gate="G$1" pin="2"/>
<wire x1="261.62" y1="104.14" x2="261.62" y2="109.22" width="0.1524" layer="91"/>
<junction x="261.62" y="109.22"/>
<pinref part="IC5" gate="A" pin="QG"/>
<pinref part="Q15" gate="1" pin="G"/>
</segment>
</net>
<net name="N$27" class="0">
<segment>
<wire x1="193.04" y1="106.68" x2="271.78" y2="106.68" width="0.1524" layer="91"/>
<pinref part="R24" gate="G$1" pin="2"/>
<wire x1="271.78" y1="104.14" x2="271.78" y2="106.68" width="0.1524" layer="91"/>
<pinref part="IC5" gate="A" pin="QH"/>
<pinref part="Q16" gate="1" pin="G"/>
<junction x="271.78" y="106.68"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="R25" gate="G$1" pin="1"/>
<wire x1="205.74" y1="149.86" x2="261.62" y2="149.86" width="0.1524" layer="91"/>
<pinref part="Q9" gate="1" pin="D"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<wire x1="215.9" y1="144.78" x2="264.16" y2="144.78" width="0.1524" layer="91"/>
<pinref part="R26" gate="G$1" pin="1"/>
<pinref part="Q10" gate="1" pin="D"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<wire x1="226.06" y1="139.7" x2="266.7" y2="139.7" width="0.1524" layer="91"/>
<pinref part="R27" gate="G$1" pin="1"/>
<pinref part="Q11" gate="1" pin="D"/>
</segment>
</net>
<net name="N$31" class="0">
<segment>
<wire x1="236.22" y1="134.62" x2="269.24" y2="134.62" width="0.1524" layer="91"/>
<pinref part="R28" gate="G$1" pin="1"/>
<pinref part="Q12" gate="1" pin="D"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<wire x1="246.38" y1="129.54" x2="271.78" y2="129.54" width="0.1524" layer="91"/>
<pinref part="R29" gate="G$1" pin="1"/>
<pinref part="Q13" gate="1" pin="D"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<wire x1="256.54" y1="124.46" x2="276.86" y2="124.46" width="0.1524" layer="91"/>
<pinref part="R30" gate="G$1" pin="1"/>
<pinref part="Q14" gate="1" pin="D"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<wire x1="266.7" y1="119.38" x2="279.4" y2="119.38" width="0.1524" layer="91"/>
<pinref part="R31" gate="G$1" pin="1"/>
<pinref part="Q15" gate="1" pin="D"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<wire x1="276.86" y1="114.3" x2="281.94" y2="114.3" width="0.1524" layer="91"/>
<pinref part="R32" gate="G$1" pin="1"/>
<pinref part="Q16" gate="1" pin="D"/>
</segment>
</net>
<net name="N$37" class="0">
<segment>
<wire x1="251.46" y1="116.84" x2="251.46" y2="111.76" width="0.1524" layer="91"/>
<wire x1="193.04" y1="111.76" x2="251.46" y2="111.76" width="0.1524" layer="91"/>
<pinref part="R22" gate="G$1" pin="2"/>
<wire x1="251.46" y1="104.14" x2="251.46" y2="111.76" width="0.1524" layer="91"/>
<junction x="251.46" y="111.76"/>
<pinref part="IC5" gate="A" pin="QF"/>
<pinref part="Q14" gate="1" pin="G"/>
</segment>
</net>
<net name="N$38" class="0">
<segment>
<wire x1="193.04" y1="124.46" x2="200.66" y2="124.46" width="0.1524" layer="91"/>
<wire x1="200.66" y1="124.46" x2="200.66" y2="142.24" width="0.1524" layer="91"/>
<pinref part="R17" gate="G$1" pin="2"/>
<wire x1="200.66" y1="104.14" x2="200.66" y2="124.46" width="0.1524" layer="91"/>
<junction x="200.66" y="124.46"/>
<pinref part="IC5" gate="A" pin="QA"/>
<pinref part="Q9" gate="1" pin="G"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="40.64" y1="195.58" x2="54.864" y2="195.58" width="0.1524" layer="91"/>
<wire x1="54.864" y1="195.58" x2="54.864" y2="172.72" width="0.1524" layer="91"/>
<pinref part="J8" gate="-2" pin="S"/>
</segment>
<segment>
<pinref part="R59" gate="G$1" pin="1"/>
<pinref part="GND3" gate="1" pin="GND"/>
<pinref part="C12" gate="G$1" pin="2"/>
<wire x1="259.08" y1="60.96" x2="259.08" y2="58.42" width="0.1524" layer="91"/>
<wire x1="259.08" y1="58.42" x2="266.7" y2="58.42" width="0.1524" layer="91"/>
<junction x="266.7" y="58.42"/>
</segment>
<segment>
<pinref part="GND5" gate="1" pin="GND"/>
<pinref part="R54" gate="G$1" pin="2"/>
<wire x1="160.02" y1="60.96" x2="170.18" y2="60.96" width="0.1524" layer="91"/>
<pinref part="Q17" gate="1" pin="S"/>
<wire x1="170.18" y1="66.04" x2="170.18" y2="60.96" width="0.1524" layer="91"/>
<junction x="170.18" y="60.96"/>
</segment>
<segment>
<pinref part="IC3" gate="A" pin="INH"/>
<wire x1="124.46" y1="121.92" x2="119.38" y2="121.92" width="0.1524" layer="91"/>
<wire x1="119.38" y1="121.92" x2="119.38" y2="167.64" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="INH"/>
<wire x1="124.46" y1="167.64" x2="119.38" y2="167.64" width="0.1524" layer="91"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="119.38" y1="115.57" x2="119.38" y2="121.92" width="0.1524" layer="91"/>
<junction x="119.38" y="121.92"/>
<pinref part="IC2" gate="A" pin="SER"/>
<wire x1="124.46" y1="195.58" x2="119.38" y2="195.58" width="0.1524" layer="91"/>
<wire x1="119.38" y1="195.58" x2="119.38" y2="167.64" width="0.1524" layer="91"/>
<junction x="119.38" y="167.64"/>
</segment>
<segment>
<wire x1="190.5" y1="142.24" x2="195.58" y2="142.24" width="0.1524" layer="91"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="190.5" y1="139.7" x2="190.5" y2="142.24" width="0.1524" layer="91"/>
<pinref part="R17" gate="G$1" pin="1"/>
<wire x1="200.66" y1="93.98" x2="195.58" y2="93.98" width="0.1524" layer="91"/>
<pinref part="R18" gate="G$1" pin="1"/>
<pinref part="R23" gate="G$1" pin="1"/>
<junction x="261.62" y="93.98"/>
<pinref part="R24" gate="G$1" pin="1"/>
<junction x="271.78" y="93.98"/>
<pinref part="R57" gate="G$1" pin="2"/>
<wire x1="281.94" y1="93.98" x2="276.86" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q18" gate="1" pin="S"/>
<wire x1="276.86" y1="86.36" x2="276.86" y2="93.98" width="0.1524" layer="91"/>
<junction x="276.86" y="93.98"/>
<wire x1="276.86" y1="93.98" x2="271.78" y2="93.98" width="0.1524" layer="91"/>
<wire x1="271.78" y1="93.98" x2="266.7" y2="93.98" width="0.1524" layer="91"/>
<pinref part="R22" gate="G$1" pin="1"/>
<junction x="251.46" y="93.98"/>
<wire x1="266.7" y1="93.98" x2="261.62" y2="93.98" width="0.1524" layer="91"/>
<wire x1="251.46" y1="93.98" x2="256.54" y2="93.98" width="0.1524" layer="91"/>
<pinref part="R21" gate="G$1" pin="1"/>
<wire x1="256.54" y1="93.98" x2="261.62" y2="93.98" width="0.1524" layer="91"/>
<wire x1="241.3" y1="93.98" x2="246.38" y2="93.98" width="0.1524" layer="91"/>
<junction x="241.3" y="93.98"/>
<pinref part="R20" gate="G$1" pin="1"/>
<junction x="231.14" y="93.98"/>
<pinref part="R19" gate="G$1" pin="1"/>
<wire x1="246.38" y1="93.98" x2="251.46" y2="93.98" width="0.1524" layer="91"/>
<wire x1="231.14" y1="93.98" x2="236.22" y2="93.98" width="0.1524" layer="91"/>
<wire x1="236.22" y1="93.98" x2="241.3" y2="93.98" width="0.1524" layer="91"/>
<wire x1="220.98" y1="93.98" x2="226.06" y2="93.98" width="0.1524" layer="91"/>
<junction x="220.98" y="93.98"/>
<wire x1="226.06" y1="93.98" x2="231.14" y2="93.98" width="0.1524" layer="91"/>
<wire x1="220.98" y1="93.98" x2="215.9" y2="93.98" width="0.1524" layer="91"/>
<wire x1="215.9" y1="93.98" x2="210.82" y2="93.98" width="0.1524" layer="91"/>
<wire x1="200.66" y1="93.98" x2="205.74" y2="93.98" width="0.1524" layer="91"/>
<wire x1="205.74" y1="93.98" x2="210.82" y2="93.98" width="0.1524" layer="91"/>
<junction x="205.74" y="93.98"/>
<junction x="200.66" y="93.98"/>
<junction x="210.82" y="93.98"/>
<pinref part="Q10" gate="1" pin="S"/>
<wire x1="205.74" y1="144.78" x2="205.74" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q11" gate="1" pin="S"/>
<wire x1="215.9" y1="139.7" x2="215.9" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q12" gate="1" pin="S"/>
<wire x1="226.06" y1="134.62" x2="226.06" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q13" gate="1" pin="S"/>
<wire x1="236.22" y1="129.54" x2="236.22" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q14" gate="1" pin="S"/>
<wire x1="246.38" y1="124.46" x2="246.38" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q15" gate="1" pin="S"/>
<wire x1="256.54" y1="119.38" x2="256.54" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q16" gate="1" pin="S"/>
<wire x1="266.7" y1="114.3" x2="266.7" y2="93.98" width="0.1524" layer="91"/>
<junction x="215.9" y="93.98"/>
<junction x="226.06" y="93.98"/>
<junction x="236.22" y="93.98"/>
<junction x="246.38" y="93.98"/>
<junction x="256.54" y="93.98"/>
<junction x="266.7" y="93.98"/>
<pinref part="R55" gate="G$1" pin="2"/>
<pinref part="Q1" gate="1" pin="S"/>
<wire x1="195.58" y1="210.82" x2="195.58" y2="154.94" width="0.1524" layer="91"/>
<junction x="195.58" y="142.24"/>
<wire x1="195.58" y1="154.94" x2="195.58" y2="149.86" width="0.1524" layer="91"/>
<wire x1="195.58" y1="149.86" x2="195.58" y2="142.24" width="0.1524" layer="91"/>
<wire x1="195.58" y1="142.24" x2="195.58" y2="93.98" width="0.1524" layer="91"/>
<pinref part="Q9" gate="1" pin="S"/>
<junction x="195.58" y="149.86"/>
<pinref part="R1" gate="G$1" pin="1"/>
<junction x="200.66" y="154.94"/>
<pinref part="R2" gate="G$1" pin="1"/>
<junction x="210.82" y="154.94"/>
<pinref part="R3" gate="G$1" pin="1"/>
<junction x="220.98" y="154.94"/>
<pinref part="R4" gate="G$1" pin="1"/>
<junction x="231.14" y="154.94"/>
<pinref part="R5" gate="G$1" pin="1"/>
<junction x="241.3" y="154.94"/>
<pinref part="R6" gate="G$1" pin="1"/>
<junction x="251.46" y="154.94"/>
<pinref part="R7" gate="G$1" pin="1"/>
<junction x="261.62" y="154.94"/>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="271.78" y1="154.94" x2="261.62" y2="154.94" width="0.1524" layer="91"/>
<wire x1="261.62" y1="154.94" x2="251.46" y2="154.94" width="0.1524" layer="91"/>
<wire x1="251.46" y1="154.94" x2="241.3" y2="154.94" width="0.1524" layer="91"/>
<wire x1="241.3" y1="154.94" x2="231.14" y2="154.94" width="0.1524" layer="91"/>
<wire x1="231.14" y1="154.94" x2="220.98" y2="154.94" width="0.1524" layer="91"/>
<wire x1="220.98" y1="154.94" x2="210.82" y2="154.94" width="0.1524" layer="91"/>
<wire x1="210.82" y1="154.94" x2="200.66" y2="154.94" width="0.1524" layer="91"/>
<wire x1="200.66" y1="154.94" x2="195.58" y2="154.94" width="0.1524" layer="91"/>
<junction x="195.58" y="154.94"/>
</segment>
</net>
<net name="OUT_2" class="0">
<segment>
<pinref part="R30" gate="G$1" pin="2"/>
<wire x1="287.02" y1="124.46" x2="307.34" y2="124.46" width="0.1524" layer="91"/>
<label x="298.45" y="125.222" size="1.778" layer="95"/>
<pinref part="J9" gate="-4" pin="S"/>
</segment>
</net>
<net name="OUT_3" class="0">
<segment>
<pinref part="R29" gate="G$1" pin="2"/>
<wire x1="281.94" y1="129.54" x2="307.34" y2="129.54" width="0.1524" layer="91"/>
<label x="298.704" y="130.302" size="1.778" layer="95"/>
<pinref part="J9" gate="-5" pin="S"/>
</segment>
</net>
<net name="OUT_4" class="0">
<segment>
<pinref part="R28" gate="G$1" pin="2"/>
<wire x1="279.4" y1="134.62" x2="307.34" y2="134.62" width="0.1524" layer="91"/>
<label x="298.45" y="135.382" size="1.778" layer="95"/>
<pinref part="J9" gate="-6" pin="S"/>
</segment>
</net>
<net name="OUT_5" class="0">
<segment>
<pinref part="R27" gate="G$1" pin="2"/>
<wire x1="276.86" y1="139.7" x2="307.34" y2="139.7" width="0.1524" layer="91"/>
<label x="298.704" y="140.462" size="1.778" layer="95"/>
<pinref part="J9" gate="-7" pin="S"/>
</segment>
</net>
<net name="OUT_7" class="0">
<segment>
<pinref part="R25" gate="G$1" pin="2"/>
<wire x1="271.78" y1="149.86" x2="307.34" y2="149.86" width="0.1524" layer="91"/>
<label x="298.45" y="150.622" size="1.778" layer="95"/>
<pinref part="J9" gate="-9" pin="S"/>
</segment>
</net>
<net name="OUT_6" class="0">
<segment>
<pinref part="R26" gate="G$1" pin="2"/>
<wire x1="274.32" y1="144.78" x2="307.34" y2="144.78" width="0.1524" layer="91"/>
<label x="298.45" y="145.542" size="1.778" layer="95"/>
<pinref part="J9" gate="-8" pin="S"/>
</segment>
</net>
<net name="BEEPER_PWM" class="0">
<segment>
<label x="137.16" y="70.104" size="1.778" layer="95"/>
<pinref part="R54" gate="G$1" pin="1"/>
<wire x1="160.02" y1="71.12" x2="154.94" y2="71.12" width="0.1524" layer="91"/>
<pinref part="Q17" gate="1" pin="G"/>
<wire x1="162.56" y1="71.12" x2="160.02" y2="71.12" width="0.1524" layer="91"/>
<junction x="160.02" y="71.12"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="R56" gate="G$1" pin="2"/>
<pinref part="SG1" gate="G$1" pin="1"/>
<wire x1="185.42" y1="78.74" x2="182.88" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+12V" class="0">
<segment>
<pinref part="SG1" gate="G$1" pin="2"/>
<wire x1="193.04" y1="78.74" x2="195.58" y2="78.74" width="0.1524" layer="91"/>
<wire x1="195.58" y1="78.74" x2="195.58" y2="66.04" width="0.1524" layer="91"/>
<wire x1="195.58" y1="66.04" x2="200.66" y2="66.04" width="0.1524" layer="91"/>
<wire x1="200.66" y1="66.04" x2="200.66" y2="68.58" width="0.1524" layer="91"/>
<pinref part="P+2" gate="1" pin="+12V"/>
</segment>
<segment>
<wire x1="307.34" y1="213.36" x2="307.34" y2="218.44" width="0.1524" layer="91"/>
<pinref part="P+4" gate="1" pin="+12V"/>
<pinref part="J8" gate="-1" pin="S"/>
</segment>
<segment>
<pinref part="R61" gate="G$1" pin="1"/>
<wire x1="297.18" y1="106.68" x2="292.1" y2="106.68" width="0.1524" layer="91"/>
<pinref part="P+3" gate="1" pin="+12V"/>
<wire x1="292.1" y1="106.68" x2="292.1" y2="109.22" width="0.1524" layer="91"/>
</segment>
</net>
<net name="CONTACTOR_PD" class="0">
<segment>
<pinref part="Q18" gate="1" pin="D"/>
<wire x1="276.86" y1="76.2" x2="276.86" y2="73.66" width="0.1524" layer="91"/>
<wire x1="276.86" y1="73.66" x2="279.4" y2="73.66" width="0.1524" layer="91"/>
<label x="277.114" y="74.93" size="1.778" layer="95"/>
<pinref part="P6" gate="-3" pin="S"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="R58" gate="G$1" pin="2"/>
<pinref part="P6" gate="-4" pin="S"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="R57" gate="G$1" pin="1"/>
<pinref part="IC7" gate="F" pin="I"/>
<wire x1="299.72" y1="68.58" x2="299.72" y2="81.28" width="0.1524" layer="91"/>
<wire x1="299.72" y1="81.28" x2="299.72" y2="93.98" width="0.1524" layer="91"/>
<pinref part="IC7" gate="D" pin="I"/>
<pinref part="IC7" gate="E" pin="I"/>
<junction x="299.72" y="81.28"/>
<wire x1="292.1" y1="93.98" x2="299.72" y2="93.98" width="0.1524" layer="91"/>
<junction x="299.72" y="93.98"/>
</segment>
</net>
<net name="N$40" class="0">
<segment>
<pinref part="IC7" gate="C" pin="I"/>
<pinref part="IC7" gate="B" pin="O"/>
<wire x1="243.84" y1="81.28" x2="246.38" y2="81.28" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$41" class="0">
<segment>
<pinref part="IC7" gate="C" pin="O"/>
<pinref part="Q18" gate="1" pin="G"/>
<wire x1="266.7" y1="81.28" x2="269.24" y2="81.28" width="0.1524" layer="91"/>
</segment>
</net>
<net name="CONTACTOR_PWM" class="0">
<segment>
<pinref part="IC7" gate="B" pin="I"/>
<wire x1="223.52" y1="81.28" x2="220.98" y2="81.28" width="0.1524" layer="91"/>
<label x="197.612" y="81.534" size="1.778" layer="95"/>
<pinref part="R55" gate="G$1" pin="1"/>
<wire x1="220.98" y1="81.28" x2="215.9" y2="81.28" width="0.1524" layer="91"/>
<wire x1="220.98" y1="83.82" x2="220.98" y2="81.28" width="0.1524" layer="91"/>
<junction x="220.98" y="81.28"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="Q8" gate="1" pin="S"/>
<wire x1="266.7" y1="154.94" x2="266.7" y2="175.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$42" class="0">
<segment>
<wire x1="205.74" y1="205.74" x2="205.74" y2="154.94" width="0.1524" layer="91"/>
<pinref part="Q2" gate="1" pin="S"/>
</segment>
</net>
<net name="N$43" class="0">
<segment>
<pinref part="Q3" gate="1" pin="S"/>
<wire x1="215.9" y1="200.66" x2="215.9" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$44" class="0">
<segment>
<pinref part="Q4" gate="1" pin="S"/>
<wire x1="226.06" y1="195.58" x2="226.06" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$45" class="0">
<segment>
<pinref part="Q5" gate="1" pin="S"/>
<wire x1="236.22" y1="190.5" x2="236.22" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$46" class="0">
<segment>
<pinref part="Q6" gate="1" pin="S"/>
<wire x1="246.38" y1="185.42" x2="246.38" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$47" class="0">
<segment>
<pinref part="Q7" gate="1" pin="S"/>
<wire x1="256.54" y1="180.34" x2="256.54" y2="157.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$48" class="0">
<segment>
<pinref part="IC3" gate="A" pin="SH/!LD"/>
<wire x1="121.92" y1="165.1" x2="121.92" y2="119.38" width="0.1524" layer="91"/>
<wire x1="121.92" y1="119.38" x2="124.46" y2="119.38" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="SH/!LD"/>
<wire x1="124.46" y1="165.1" x2="121.92" y2="165.1" width="0.1524" layer="91"/>
<wire x1="111.76" y1="116.84" x2="121.92" y2="116.84" width="0.1524" layer="91"/>
<wire x1="121.92" y1="116.84" x2="121.92" y2="119.38" width="0.1524" layer="91"/>
<junction x="121.92" y="119.38"/>
<pinref part="IC7" gate="A" pin="O"/>
</segment>
</net>
<net name="N$39" class="0">
<segment>
<pinref part="R56" gate="G$1" pin="1"/>
<wire x1="172.72" y1="78.74" x2="170.18" y2="78.74" width="0.1524" layer="91"/>
<pinref part="Q17" gate="1" pin="D"/>
<wire x1="170.18" y1="78.74" x2="170.18" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$49" class="0">
<segment>
<wire x1="109.22" y1="124.46" x2="109.22" y2="162.56" width="0.1524" layer="91"/>
<wire x1="109.22" y1="162.56" x2="109.22" y2="208.28" width="0.1524" layer="91"/>
<wire x1="109.22" y1="124.46" x2="78.74" y2="124.46" width="0.1524" layer="91"/>
<pinref part="R53" gate="G$1" pin="2"/>
<wire x1="78.74" y1="124.46" x2="71.12" y2="124.46" width="0.1524" layer="91"/>
<wire x1="71.12" y1="124.46" x2="60.96" y2="124.46" width="0.1524" layer="91"/>
<wire x1="60.96" y1="124.46" x2="50.8" y2="124.46" width="0.1524" layer="91"/>
<wire x1="50.8" y1="124.46" x2="50.8" y2="121.92" width="0.1524" layer="91"/>
<pinref part="R52" gate="G$1" pin="2"/>
<wire x1="60.96" y1="121.92" x2="60.96" y2="124.46" width="0.1524" layer="91"/>
<pinref part="R51" gate="G$1" pin="2"/>
<wire x1="71.12" y1="121.92" x2="71.12" y2="124.46" width="0.1524" layer="91"/>
<pinref part="R50" gate="G$1" pin="2"/>
<wire x1="78.74" y1="121.92" x2="78.74" y2="124.46" width="0.1524" layer="91"/>
<junction x="78.74" y="124.46"/>
<junction x="71.12" y="124.46"/>
<junction x="60.96" y="124.46"/>
<pinref part="R49" gate="G$1" pin="2"/>
<wire x1="50.8" y1="160.02" x2="50.8" y2="162.56" width="0.1524" layer="91"/>
<wire x1="50.8" y1="162.56" x2="58.42" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R42" gate="G$1" pin="2"/>
<wire x1="58.42" y1="162.56" x2="66.04" y2="162.56" width="0.1524" layer="91"/>
<wire x1="66.04" y1="162.56" x2="73.66" y2="162.56" width="0.1524" layer="91"/>
<wire x1="73.66" y1="162.56" x2="81.28" y2="162.56" width="0.1524" layer="91"/>
<wire x1="81.28" y1="162.56" x2="88.9" y2="162.56" width="0.1524" layer="91"/>
<wire x1="88.9" y1="162.56" x2="96.52" y2="162.56" width="0.1524" layer="91"/>
<wire x1="96.52" y1="162.56" x2="104.14" y2="162.56" width="0.1524" layer="91"/>
<wire x1="104.14" y1="162.56" x2="109.22" y2="162.56" width="0.1524" layer="91"/>
<wire x1="104.14" y1="160.02" x2="104.14" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R43" gate="G$1" pin="2"/>
<wire x1="96.52" y1="160.02" x2="96.52" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R44" gate="G$1" pin="2"/>
<wire x1="88.9" y1="160.02" x2="88.9" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R45" gate="G$1" pin="2"/>
<wire x1="81.28" y1="160.02" x2="81.28" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R46" gate="G$1" pin="2"/>
<wire x1="73.66" y1="160.02" x2="73.66" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R47" gate="G$1" pin="2"/>
<wire x1="66.04" y1="160.02" x2="66.04" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R48" gate="G$1" pin="2"/>
<wire x1="58.42" y1="160.02" x2="58.42" y2="162.56" width="0.1524" layer="91"/>
<pinref part="R41" gate="G$1" pin="2"/>
<wire x1="50.8" y1="205.74" x2="50.8" y2="208.28" width="0.1524" layer="91"/>
<wire x1="50.8" y1="208.28" x2="58.42" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R34" gate="G$1" pin="2"/>
<wire x1="58.42" y1="208.28" x2="66.04" y2="208.28" width="0.1524" layer="91"/>
<wire x1="66.04" y1="208.28" x2="73.66" y2="208.28" width="0.1524" layer="91"/>
<wire x1="73.66" y1="208.28" x2="81.28" y2="208.28" width="0.1524" layer="91"/>
<wire x1="81.28" y1="208.28" x2="88.9" y2="208.28" width="0.1524" layer="91"/>
<wire x1="88.9" y1="208.28" x2="96.52" y2="208.28" width="0.1524" layer="91"/>
<wire x1="96.52" y1="208.28" x2="104.14" y2="208.28" width="0.1524" layer="91"/>
<wire x1="104.14" y1="208.28" x2="109.22" y2="208.28" width="0.1524" layer="91"/>
<wire x1="104.14" y1="205.74" x2="104.14" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R35" gate="G$1" pin="2"/>
<wire x1="96.52" y1="205.74" x2="96.52" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R36" gate="G$1" pin="2"/>
<wire x1="88.9" y1="205.74" x2="88.9" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R37" gate="G$1" pin="2"/>
<wire x1="81.28" y1="205.74" x2="81.28" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R38" gate="G$1" pin="2"/>
<wire x1="73.66" y1="205.74" x2="73.66" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R39" gate="G$1" pin="2"/>
<wire x1="66.04" y1="205.74" x2="66.04" y2="208.28" width="0.1524" layer="91"/>
<pinref part="R40" gate="G$1" pin="2"/>
<wire x1="58.42" y1="205.74" x2="58.42" y2="208.28" width="0.1524" layer="91"/>
<junction x="58.42" y="208.28"/>
<junction x="66.04" y="208.28"/>
<junction x="73.66" y="208.28"/>
<junction x="81.28" y="208.28"/>
<junction x="88.9" y="208.28"/>
<junction x="96.52" y="208.28"/>
<junction x="104.14" y="208.28"/>
<junction x="109.22" y="162.56"/>
<junction x="104.14" y="162.56"/>
<junction x="96.52" y="162.56"/>
<junction x="88.9" y="162.56"/>
<junction x="81.28" y="162.56"/>
<junction x="73.66" y="162.56"/>
<junction x="66.04" y="162.56"/>
<junction x="58.42" y="162.56"/>
<pinref part="R60" gate="G$1" pin="2"/>
<wire x1="132.08" y1="208.28" x2="109.22" y2="208.28" width="0.1524" layer="91"/>
<junction x="109.22" y="208.28"/>
</segment>
</net>
<net name="N$50" class="0">
<segment>
<pinref part="J9" gate="-1" pin="S"/>
<pinref part="R61" gate="G$1" pin="2"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
</compatibility>
</eagle>
