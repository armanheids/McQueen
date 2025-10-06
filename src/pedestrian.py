import carla
import random
import time

def main():
    # اتصال به سرور CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # دریافت world
    world = client.get_world()
    
    # تنظیمات برای synchronous mode (اختیاری ولی توصیه می‌شود)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    # دریافت spectator
    spectator = world.get_spectator()
    
    # دریافت blueprint library
    blueprint_library = world.get_blueprint_library()
    
    # انتخاب blueprint عابر
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    
    # انتخاب spawn point
    spawn_points = world.get_map().get_spawn_points()
    
    walker = None
    walker_controller = None
    
    try:
        if spawn_points:
            spawn_point = random.choice(spawn_points)
            
            # تنظیم موقعیت spawn عابر
            walker_spawn_point = carla.Transform(
                carla.Location(
                    x=spawn_point.location.x + 5,
                    y=spawn_point.location.y,
                    z=spawn_point.location.z + 1
                ),
                carla.Rotation(yaw=spawn_point.rotation.yaw)
            )
            
            # ایجاد عابر
            walker = world.spawn_actor(walker_bp, walker_spawn_point)
            print(f"✓ عابر ایجاد شد با ID: {walker.id}")
            
            # صبر کنید تا عابر spawn شود
            world.tick()
            time.sleep(0.5)
            
            # ایجاد AI controller
            walker_controller_bp = blueprint_library.find('controller.ai.walker')
            walker_controller = world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
            print(f"✓ کنترلر ایجاد شد با ID: {walker_controller.id}")
            
            # صبر کنید تا controller متصل شود
            world.tick()
            time.sleep(0.5)
            
            # شروع کنترلر
            walker_controller.start()
            print("✓ کنترلر فعال شد")
            
            world.tick()
            
            # تعیین مقصد
            destination = carla.Location(
                x=walker_spawn_point.location.x - 10,
                y=walker_spawn_point.location.y,
                z=walker_spawn_point.location.z
            )
            
            print(f"موقعیت شروع: ({walker_spawn_point.location.x:.2f}, {walker_spawn_point.location.y:.2f})")
            print(f"موقعیت مقصد: ({destination.x:.2f}, {destination.y:.2f})")
            
            # ارسال دستور حرکت
            walker_controller.go_to_location(destination)
            walker_controller.set_max_speed(2.0)  # افزایش سرعت
            
            print("✓ دستور حرکت ارسال شد")
            
            world.tick()
            
            # تنظیم دوربین
            camera_transform = carla.Transform(
                carla.Location(
                    x=walker_spawn_point.location.x - 8,
                    y=walker_spawn_point.location.y + 15,
                    z=walker_spawn_point.location.z + 8
                ),
                carla.Rotation(pitch=-30, yaw=-70)
            )
            spectator.set_transform(camera_transform)
            
            print("\n⏳ در حال مشاهده حرکت عابر...")
            print("(برای توقف Ctrl+C را فشار دهید)\n")
            
            # حلقه اصلی برای نمایش موقعیت
            for i in range(300):  # 15 ثانیه (با 0.05s tick)
                world.tick()
                
                if i % 20 == 0:  # هر 1 ثانیه
                    walker_location = walker.get_location()
                    velocity = walker.get_velocity()
                    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                    
                    print(f"زمان: {i*0.05:.1f}s | موقعیت: ({walker_location.x:.2f}, {walker_location.y:.2f}) | سرعت: {speed:.2f} m/s")
                
                time.sleep(0.05)
            
    except Exception as e:
        print(f"❌ خطا: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # پاکسازی
        print("\n🧹 در حال پاکسازی...")
        
        if walker_controller is not None:
            walker_controller.stop()
            walker_controller.destroy()
            print("✓ کنترلر پاک شد")
        
        if walker is not None:
            walker.destroy()
            print("✓ عابر پاک شد")
        
        # بازگشت به حالت asynchronous
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("✓ تنظیمات بازگردانده شد")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n\n⛔ لغو شد توسط کاربر')
