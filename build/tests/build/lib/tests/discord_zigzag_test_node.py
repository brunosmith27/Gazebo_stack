import os
import discord
from discord.ext import commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DiscordZigzagBotNode(Node):
    def __init__(self):
        super().__init__('discord_zigzag_test_node')

        intents = discord.Intents.default()
        intents.message_content = True
        self.bot = commands.Bot(command_prefix="!", intents=intents)

        self.WORKSPACE_DIR = "/home/user/abd_ws_2"
        self.perimeter_dir = os.path.join(self.WORKSPACE_DIR, "src/tests/tests/perimeter")
        self.perimeter_name = None

        self.cmd_pub = self.create_publisher(String, '/perimeter_cmd', 10)
        self.name_pub = self.create_publisher(String, '/perimeter_name', 10)
        self.nav_trigger_pub = self.create_publisher(String, '/stanley_start_cmd', 10)

        self.setup_bot_commands()

    def send_command(self, command_char):
        msg = String()
        msg.data = command_char
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published: {command_char}")

    def send_name(self, name):
        msg = String()
        msg.data = name
        self.name_pub.publish(msg)
        self.get_logger().info(f"Published perimeter name: {name}")

    def send_navigation_trigger(self, action="start"):
        msg = String()
        msg.data = action
        self.nav_trigger_pub.publish(msg)
        self.get_logger().info(f"Published navigation trigger: {action}")

    def setup_bot_commands(self):
        @self.bot.event
        async def on_ready():
            print(f"🤖 Bot is ready. Logged in as {self.bot.user}")

        @self.bot.command()
        async def start_test(ctx):
            await ctx.send("🚀 Starting ZigZag test...")
            await ctx.send("🧠 Launching sensor nodes (./start_abd.sh)...")
            await ctx.send("📍 Now run `!start_perimeter` to begin perimeter recording manually.")

        @self.bot.command()
        async def start_perimeter(ctx):
            await ctx.send("❓ What is the name of this perimeter?")

            def check(m):
                return m.author == ctx.author and m.channel == ctx.channel

            try:
                response = await self.bot.wait_for('message', check=check, timeout=60)
                self.perimeter_name = response.content.strip()
                self.send_name(self.perimeter_name)

                folder_path = os.path.join(self.perimeter_dir, self.perimeter_name)
                os.makedirs(folder_path, exist_ok=True)

                await ctx.send(f"📁 Perimeter name set to `{self.perimeter_name}`")
                await ctx.send("📍 Starting perimeter recording node...")
                await ctx.send(f"✅ Data will be saved in: `{folder_path}`")
                await ctx.send("📀 Use `!send_y`, `!send_n`, and `!send_e` to control perimeter recording.")

            except Exception:
                await ctx.send("⏳ Timeout! Please run `!start_perimeter` again.")

        @self.bot.command()
        async def send_y(ctx):
            await ctx.send("✏️ Sending command: y (start recording)...")
            self.send_command('y')

        @self.bot.command()
        async def send_n(ctx):
            await ctx.send("🛑 Sending command: n (stop drawing)...")
            self.send_command('n')
            if self.perimeter_name:
                await ctx.send(f"📂 Data saved to `{os.path.join(self.perimeter_dir, self.perimeter_name)}`")

        @self.bot.command()
        async def send_e(ctx):
            await ctx.send("🚪 Sending command: e (exit node)...")
            self.send_command('e')
            await ctx.send("📁 Now run `!generate_path` to process GPS and create zigzag path.")

        @self.bot.command()
        async def generate_path(ctx):
            await ctx.send("🧙‍♂️ Generating zigzag paths from recorded perimeter...")
            await ctx.send("✅ Zigzag path generated.")
            await ctx.send("🤖 Ready to begin autonomous navigation. Type `!confirm_start` to launch robot.")
            await ctx.send("💡 To stop the robot later, use the command: `!stop_navigation`.")

        @self.bot.command()
        async def confirm_start(ctx):
            await ctx.send("🗂 Which perimeter would you like to navigate?")

            def check(m):
                return m.author == ctx.author and m.channel == ctx.channel

            try:
                response = await self.bot.wait_for('message', check=check, timeout=60)
                chosen_perimeter = response.content.strip()
                path_dir = os.path.join(self.perimeter_dir, chosen_perimeter)

                if not os.path.exists(path_dir):
                    await ctx.send("❌ That perimeter does not exist! Please check the name.")
                    return

                self.perimeter_name = chosen_perimeter
                self.send_name(chosen_perimeter)
                self.send_command('start')
                self.send_navigation_trigger("start")

                await ctx.send(f"📆 Loading perimeter `{chosen_perimeter}` from `{path_dir}`")
                await ctx.send("🛠 Launching navigation_node to start robot...")
                await ctx.send("✅ Robot is now navigating autonomously.")
                await ctx.send("📴 To stop the robot at any time, type `!stop_navigation`.")

            except Exception:
                await ctx.send("⏳ Timeout! Please run `!confirm_start` again.")

        @self.bot.command()
        async def stop_navigation(ctx):
            self.send_navigation_trigger("stop")
            await ctx.send("🛑 Stop command sent to navigation node.")
            await ctx.send("✅ If robot doesn't stop, confirm the navigation node is subscribed to `/stanley_start_cmd` and checks `start/stop` state.")
            await ctx.send("💬 You can also re-send `!stop_navigation` again if needed.")

    def run(self, token):
        self.bot.run(token)


def main(args=None):
    rclpy.init(args=args)
    node = DiscordZigzagBotNode()
    try:
        node.run("MTM3ODA2NjgyOTQwMjA1MDYzMA.GPC-c1.NdYLUzjSfH1jP4-dBZaSSX7SZ1SzYlCE2GrykY")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
