   M S B A M L   `   `   `  ����O  KKinectMotionAnalyzer, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null;"clr-namespace:KinectMotionAnalyzerKinectMotionAnalyzer  5   	   ,    &KinectMotionAnalyzer.UI.ExerciseWindow   5   	   W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   \ XMicrosoft.Kinect.Toolkit.Controls, Version=0.0.0.0, Culture=neutral, PublicKeyToken=nullS OMicrosoft.Kinect.Toolkit, Version=1.7.0.0, Culture=neutral, PublicKeyToken=null2k(http://schemas.microsoft.com/kinect/2013   5   	   <d2http://schemas.microsoft.com/expression/blend/2008   5   	   0local"clr-namespace:KinectMotionAnalyzer   5   	   Fmc;http://schemas.openxmlformats.org/markup-compatibility/2006   5   	   -   5        $	��640��5	   	   $	��480��5
   	     � Title$  ExerciseWindow��5   	    � 
ResizeMode$ 	CanResize=�6    .��5      (�  (� Stretch$
 Fill=�6   .��5   
   �  � ShowGridLines . .+�5      4  .Microsoft.Kinect.Toolkit.Controls.KinectRegion  -   5          Name$ kinectRegion��5         CursorPressingColor$ Black��63      IsCursorVisible . .��5      � -   5         �Name$ 
LayoutRoot��5       . .+�5      �� -   5        $��color_disp_img��5      $
��Fill=�62   �� 5      -   5        $��ske_disp_img��5      $
��Fill=�60   � 5      -   5        $ kinectRegionGrid��5       .  � RowDefinitions 5      �� 5      $
��Auto �6,   �� 5      	 � ColumnDefinitions	 5      �� 5      $
��Auto �6/   �� 5      �� 5      $
��Auto �6/   .+�5       :  4Microsoft.Kinect.Toolkit.Controls.KinectCircleButton  -   5        
  Name$
 RecordButton��5    /   $��77��6`   $��0��6l   $��0��6y   .��5!      �� -   5         ��Name$ recordButtonImage��5!   $   $@��:/KinectMotionAnalyzer;component/Resources/record_start.png��6?   5"        5#      -	   5        $
 AnglesButton��5#   /   $��77��6`   $��0��6l   $��2��6y   .��5$      �� -
   5        $ angleButtonImage��5$   $   $?��9/KinectMotionAnalyzer;component/Resources/angleButton.png��6>   5%      5;      5<      5=      5>      5@      5B      