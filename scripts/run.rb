require 'orocos'
require 'vizkit'

Orocos.initialize

Orocos::run "kinect2::Task" => "task", "image_preprocessing::DepthImage2Pointcloud" => "converter" do
    task = Orocos::TaskContext.get "task"
    converter = Orocos::TaskContext.get "converter"
    task.color_frame.connect_to converter.color_frame, :type => :buffer, :size => 50
    task.depth_frame.connect_to converter.frame, :type => :buffer, :size => 50
    task.ir_frame.connect_to converter.color_frame, :type => :buffer, :size => 50
    task.configure
    task.start
#    converter.configure
#    converter.start
    Vizkit.display task
    Vizkit.display task.color_frame
#    Vizkit.display converter 
    Vizkit.exec
	Readline::readline("Press ENTER to exit\n")do
	end
end
