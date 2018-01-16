/*
Hexi Quick Start
================
This is a quick tour of all the most important things you need to
know to set up and start using Hexi. Use this same model
for structuring your own applications and games. Thanks
to Hexi's streamlined, modular design, you can create a complex interactive
application like this in less than 50 lines of compact and readable code.
Hexi's Application Architecture is made up of four main parts:
1. Setting up and starting Hexi.
2. The `load` function, that will run while your files are loading.
3. The `setup` function, which initializes your game objects, variables and sprites.
4. The `play` function, which is your game or application logic that runs in a loop.
This simple model is all you need to create any kind of game or application.
You can use it as the starting template for your own projects, and this same
basic model can scale to any size.
Take a look at the code ahead to see how it all works.
*/

/*
1. Setting up and starting Hexi
-------------------------------
*/

//Create an array of files you want to load. If you don't need to load
//any files, you can leave this out. Hexi lets you load a wide variety
//of files: images, texture atlases, bitmap fonts, ordinary font files, and
//sounds
let thingsToLoad = [
  "images/car_0.png",
];

//Initialize Hexi with the `hexi` function. It has 5 arguments,
//although only the first 3 are required:
//a. Canvas width.
//b. Canvas height.
//c. The `setup` function.
//d. The `thingsToLoad` array you defined above. This is optional.
//e. The `load` function. This is also optional.
//If you skip the last two arguments, Hexi will skip the loading
//process and jump straight to the `setup` function.
let g = hexi(512, 512, setup, thingsToLoad, load);

//Optionally Set the frames per second at which the game logic loop should run.
//(Sprites will be rendered independently, with interpolation, at full 60 or 120 fps)
//If you don't set the `fps`, Hexi will default to an fps of 60
g.fps = 30;

//Optionally add a border and set the background color
//g.border = "2px red dashed";
//g.backgroundColor = 0x000000;

//Optionally scale and align the canvas inside the browser window
g.scaleToWindow();

//Start Hexi. This is important - without this line of code, Hexi
//won't run!
g.start();


/*
2. Loading Files
----------------
*/

//The `load` function will run while assets are loading. This is the
//same `load` function you assigned as Hexi's 4th initialization argument.
//Its optional. You can leave it out if you don't have any files to
//load, or you don't need to monitor their loading progress

function load() {

  //Display the file currently being loaded
  console.log(`loading: ${g.loadingFile}`);

  //Display the percentage of files currently loaded
  console.log(`progress: ${g.loadingProgress}`);

  //Add an optional loading bar.
  g.loadingBar();

  //This built-in loading bar is fine for prototyping, but I
  //encourage to to create your own custom loading bar using Hexi's
  //`loadingFile` and `loadingProgress` values. See the `loadingBar`
  //and `makeProgressBar` methods in Hexi's `core.js` file for ideas
}


/*
3. Initialize and Set up your game objects
------------------------------------------
*/

//Declare any variables that need to be used in more than one function
let cars;

//The `setup` function will run when all the assets have loaded. This
//is the `setup` function you assigned as Hexi's 3rd argument. It's
//mandatory - every Hexi application has to have a `setup` function
//(although you can give it any name you want)

function setup() {

  //Create a `group` called `cats` to store all the cats
  cars = g.group();

  //Create a function to make a cat sprite. `makeCat` takes two arguments:
  //the `x` and `y` screen position values where the cat should start.
  //As you'll see ahead, this function is going to be called by Hexi's
  //`pointer` object each time it's clicked.
  let makeCar = (x, y) => {

    //Create the cat sprite. Supply the `sprite` method with
    //the name of the loaded image that should be displayed
    let car = g.sprite("images/car_0.png");

    //Hexi exposes Pixi (the 2D rendering engine) as a top level global object called `PIXI`. That
    //means you can use the `PIXI` object to write any low-level Pixi code,
    //directly in your Hexi application. All of Hexi's sprites are just
    //ordinary Pixi sprites under the hood, so any Pixi code you write to modify
    //them will work

    //Set the cat's position
    car.setPosition(x, y);

    //Add the cat to the `cats` group
    cars.addChild(car);
  };

  g.pointer.tap = () => {

    //Supply `makeCat` with the pointer's `x` and `y` coordinates.
    makeCar(g.pointer.x, g.pointer.y);
  };
  g.state = play;

}


/*
4. The game logic
------------------
*/

//The `play` function is called in a continuous loop, at whatever fps
//(frames per second) value you set. This is your *game logic loop*. (The
//render loop will be run by Hexi in the background at the maximum fps
//your system can handle.) You can pause Hexi's game loop at any time
//with the `pause` method, and restart it with the `resume` method

function play() {

  //Loop through all of the cats to make them move and bounce off the
  //edges of the stage
  cars.children.forEach(car => {

    //Check for a collision between the cat and the stage boundaries
    //using Hexi's `contain` method. Setting `true` as the third
    //argument will make the cat bounce when it hits the stage
    //boundaries
    car.x+=1

    //Here's what `move` is actually doing under the hood:
    //cat.x += cat.vx;
    //cat.y += cat.vy;
  });
}

/*
With this basic Hexi architecture, you can create anything. Just set
Hexi's `state` property to any other function to switch the
behaviour of your application. Here's how:
   g.state = anyStateFunction;
Write as many state functions as you need.
If it's a small project, you can keep all these functions in one file. But,
for a big project, load your functions from
external JS files as you need them. Use any module system you
prefer, like ES6 modules, CommonJS, AMD, or good old HTML `<script>` tags.
This simple architectural model can scale to any size, and is the only
architectural model you need to know. Keep it simple and stay happy!
*/