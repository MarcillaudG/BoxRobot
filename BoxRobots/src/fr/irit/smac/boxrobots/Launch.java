package fr.irit.smac.boxrobots;

public class Launch {

	public static void main(String[] args) {
		World env = new World();
		
		Storehouse _storehouse = new Storehouse(env);
		new RobotViewer(){
			protected void onInitialConfiguration() {
				this.storehouse = _storehouse;
			}
		}.start();

	}

}
