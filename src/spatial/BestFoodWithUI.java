package spatial;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.geom.Line2D;

import javax.swing.JFrame;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.continuous.ContinuousPortrayal2D;
import sim.portrayal.simple.MovablePortrayal2D;
import sim.portrayal.simple.OrientedPortrayal2D;
import sim.portrayal.simple.TrailedPortrayal2D;
import sim.util.MutableDouble2D;
import spatial.BFAgent.CommitmentState;

public class BestFoodWithUI extends GUIState {
	
	public Display2D display;
	public JFrame displayFrame;
	ContinuousPortrayal2D arenaPortrayal = new ContinuousPortrayal2D();
	ContinuousPortrayal2D circularPortrayal = new ContinuousPortrayal2D();
	ContinuousPortrayal2D trailsPortrayal = new ContinuousPortrayal2D(); 

	public static void main(String[] args) {
		String propFile = (args.length > 0)? args[0] : BestFood.DEFAULT_PROPERTIES_FILE; 
		BestFoodWithUI ui = new BestFoodWithUI(propFile);
		Console console = new Console(ui);
		console.setVisible(true);
	}

	public BestFoodWithUI(String propFile) {
		super(new BestFood(System.currentTimeMillis(), propFile ));
	}

	public BestFoodWithUI(SimState state){
		super(state);
	}

	public static String getName() { return "Collective Decision through Cross-Inhibition in Circular Arena"; }
	
	
	// code for the Inspector of the whole Model
	public Object getSimulationInspectedObject() { return state; }
	
	public Inspector getInspector(){
		Inspector i = super.getInspector();
		i.setVolatile(true);
		return i;
	}

	public void start()
	{
		super.start();		
		setupPortrayals();
	}

	public void load(SimState state)
	{
		super.load(state);
		setupPortrayals();
	}
	
	public SimplePortrayal2D makeAgent(){
//		return new OvalPortrayal2D(0.5){
//			private static final long serialVersionUID = 1L;
//
//			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
//			{
//				//info.draw.width = 1;
//				//info.draw.height = 1;
//				paint = new Color(255, 0, 0);
//				super.draw(object, graphics, info);
//			}
//		};
		
		return 	new TrailedPortrayal2D(
                	this,
                	new OrientedPortrayal2D( new SimplePortrayal2D(), 0, 0.5, new Color(255, 0, 0),
                			OrientedPortrayal2D.SHAPE_COMPASS){
                		private static final long serialVersionUID = 1L;
                		public void draw(Object object, Graphics2D graphics, DrawInfo2D info){
                			CommitmentState commitmentState = ((BFAgent)object).commitmentState; 
                			paint = (commitmentState == CommitmentState.UNCOMMITTED)? new Color(255, 255, 255) : 
                				(commitmentState == CommitmentState.OPTION_1)? new Color(150, 0, 0) : new Color(0, 100, 0) ;

                				if ( ((BFAgent)object).justAbandoned){
                					paint = new Color(0, 0, 0);
                				}

                				super.draw(object, graphics, info);

                				if ( ((BestFood)state).isDrawVectors() ){
                					double ratio = info.draw.x / ((BFAgent)object).myPosition.x;

                					/** draw line for vector to food */
                					Line2D line;
                					graphics.setStroke(	new BasicStroke(1) );
                					//if ( ((ForagerAgent)object).behaviouralState != BehaviouralState.RANDOM_WALK ) {
                					MutableDouble2D vect = new MutableDouble2D(((BFAgent)object).getVectorFromMeToFood());
                					vect.rotate( ((BFAgent)object).orientation2D() );
                					line = new Line2D.Double(info.draw.x, info.draw.y, 
                							(vect.x * ratio + info.draw.x),
                							(vect.y * ratio + info.draw.y));
                					graphics.draw(line);
                					//}

                					/** draw line for vector to nest */
                					paint = new Color(150, 150, 255);
                					graphics.setPaint(paint);
                					MutableDouble2D vectNest = new MutableDouble2D(((BFAgent)object).getVectorFromMeToNest());
                					vectNest.rotate( ((BFAgent)object).orientation2D() );
                					line = new Line2D.Double(info.draw.x, info.draw.y, 
                							(vectNest.x * ratio + info.draw.x),
                							(vectNest.y * ratio + info.draw.y));
                					graphics.draw(line);
                				}
//									
//									/** draw line for obstacle avoidance vector */
//									paint = new Color(0, 0, 0);
//									graphics.setPaint(paint);
//									vect = new MutableDouble2D(((Insect)object).getObstacleAvoidanceVector());
//									// vect.rotate( ((Insect)object).orientation2D() );
//									line = new Line2D.Double(info.draw.x, info.draw.y, 
//											(vect.x * ratio + info.draw.x),
//											(vect.y * ratio + info.draw.y));
//									graphics.draw(line);
									
									/** draw sensing circle */
//									paint = new Color(0, 0, 0);
//									graphics.setPaint(paint);
//									double size = ((NestSelection)state).localSensing * ratio;
//									Ellipse2D sensing = new Ellipse2D.Double(info.draw.x - size , info.draw.y - size, size*2, size*2);
//									graphics.draw(sensing);
									
//									/** draw line to convincer */
//									paint = new Color(180, 180, 180);
//									graphics.setPaint(paint);
//									for (Double2D convPos: ((Insect)object).convincerPosition){
//										line = new Line2D.Double(info.draw.x, info.draw.y,
//												(convPos.x * ratio), (convPos.y * ratio));
//										graphics.draw(line);
//									}
								}
                	},
                    trailsPortrayal, 100);
		
	}

	public void setupPortrayals()
	{
		BestFood currentState = (BestFood) state;
		// tell the portrayals what to portray and how to portray them
		// print the circular arena
		circularPortrayal.setField( currentState.circularArena );
		// print the insects
		trailsPortrayal.setField( currentState.arena );
		arenaPortrayal.setField( currentState.arena );
		for(int x=0; x < currentState.arena.allObjects.numObjs; x++) {
			SimplePortrayal2D p = makeAgent();
			arenaPortrayal.setPortrayalForObject(currentState.arena.allObjects.objs[x], new MovablePortrayal2D(p));
			trailsPortrayal.setPortrayalForObject(currentState.arena.allObjects.objs[x], p);
		}
		// arenaPortrayal.setPortrayalForAll( makeAgent() );
		// reschedule the displayer
		display.reset();
		display.setBackdrop(Color.white);
		// redraw the display
		display.repaint();
	}

	public void init(Controller c)
	{
		super.init(c);
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
		int height = (int) Math.round( screenSize.getHeight() * 0.8 );
		display = new Display2D(height,height,this);
		display.setClipping(false);
		displayFrame = display.createFrame();
		displayFrame.setTitle("Arena view");
		c.registerFrame(displayFrame); // so the frame appears in the "Display" list
		displayFrame.setVisible(true);
		display.attach( circularPortrayal, "Circular Arena" );
		display.attach( trailsPortrayal, "Trails" );
		display.attach( arenaPortrayal,  "Full Arena" );
	}

	public void quit()
	{
		super.quit();
		if (displayFrame!=null) displayFrame.dispose();
		displayFrame = null;
		display = null;
	}

}
