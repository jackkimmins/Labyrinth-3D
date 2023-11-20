#include <SDL.h>
#include <emscripten.h>
#include <cmath>
#include <chrono>
#include <iostream>

#define WIDTH 1920
#define HEIGHT 1080
#define MAP_SIZE 8
#define FOV 70.0

double playerZ = 0.0;  // Player's vertical position (Z-axis in 3D)
double verticalSpeed = 0.0;  // Speed at which the player is moving vertically
bool isJumping = false;
const double GRAVITY = -0.05;  // Simulate gravitational pull
const double JUMP_FORCE = 3.0;  // Initial upward force when jumping

// World map
int worldMap[MAP_SIZE][MAP_SIZE] = {
    {1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,1},
    {1,0,1,0,1,1,1,1},
    {1,0,1,0,0,0,0,1},
    {1,0,0,0,1,0,1,1},
    {1,0,1,0,0,0,0,1},
    {1,0,0,0,1,0,0,1},
    {1,1,1,1,1,1,1,1}
};

// Player position
double posX = 2.5, posY = 2.5;
double dirX = -1.0, dirY = 0.0; // initial direction vector
double planeX = 0.0;
double planeY = tan(FOV * 0.5 * M_PI / 180.0);

double moveSpeed = 0.0015;
double rotSpeed = 0.0014;

SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;

// Timing for deltatime
double lastUpdate = clock();
double deltaTime = (clock() - lastUpdate) / 1000.0f; 

void Render()
{
    // Clear screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (int x = 0; x < WIDTH; x++)
    {
        double cameraX = 2.0 * x / double(WIDTH) - 1.0;
        double rayDirX = dirX + planeX * cameraX;
        double rayDirY = dirY + planeY * cameraX;

        int mapX = int(posX);
        int mapY = int(posY);

        double sideDistX;
        double sideDistY;

        double deltaDistX = std::abs(1 / rayDirX);
        double deltaDistY = std::abs(1 / rayDirY);
        double perpWallDist;

        int stepX;
        int stepY;

        int hit = 0;
        int side;

        if (rayDirX < 0)
        {
            stepX = -1;
            sideDistX = (posX - mapX) * deltaDistX;
        }
        else
        {
            stepX = 1;
            sideDistX = (mapX + 1.0 - posX) * deltaDistX;
        }

        if (rayDirY < 0)
        {
            stepY = -1;
            sideDistY = (posY - mapY) * deltaDistY;
        }
        else
        {
            stepY = 1;
            sideDistY = (mapY + 1.0 - posY) * deltaDistY;
        }

        while (hit == 0)
        {
            if (sideDistX < sideDistY)
            {
                sideDistX += deltaDistX;
                mapX += stepX;
                side = 0;
            }
            else
            {
                sideDistY += deltaDistY;
                mapY += stepY;
                side = 1;
            }

            if (worldMap[mapX][mapY] > 0) hit = 1;
        }

        if (side == 0) perpWallDist = (mapX - posX + (1 - stepX) / 2) / rayDirX;
        else perpWallDist = (mapY - posY + (1 - stepY) / 2) / rayDirY;

        int lineHeight = int(HEIGHT / perpWallDist);

        int drawStart = -lineHeight / 2 + HEIGHT / 2 + playerZ;
        if(drawStart < 0) drawStart = 0;
        int drawEnd = lineHeight / 2 + HEIGHT / 2 + playerZ;
        if(drawEnd >= HEIGHT) drawEnd = HEIGHT - 1;

        int color = 0xFF0000;
        if (worldMap[mapX][mapY] == 1) color = 0x00FF00;
        if (worldMap[mapX][mapY] == 2) color = 0x0000FF;
        if (worldMap[mapX][mapY] == 3) color = 0xFFFF00;
        if (worldMap[mapX][mapY] == 4) color = 0xFF00FF;
        if (side == 1) color = color / 2;

        SDL_SetRenderDrawColor(renderer, (color & 0xFF0000) >> 16, (color & 0x00FF00) >> 8, color & 0x0000FF, 255);
        SDL_RenderDrawLine(renderer, x, drawStart, x, drawEnd);
    }

    // Update screen
    SDL_RenderPresent(renderer);
}

void Update()
{
    const Uint8* state = SDL_GetKeyboardState(NULL);



    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        if (e.type == SDL_QUIT || (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE))
        {
            SDL_DestroyWindow(window);
            SDL_Quit();
            exit(0);
        }
    }

    if (state[SDL_SCANCODE_UP] || state[SDL_SCANCODE_W])
    {
        posX += dirX * moveSpeed * deltaTime;
        posY += dirY * moveSpeed * deltaTime;
    }
    if (state[SDL_SCANCODE_DOWN] || state[SDL_SCANCODE_S])
    {
        posX -= dirX * moveSpeed * deltaTime;
        posY -= dirY * moveSpeed * deltaTime;
    }
    if (state[SDL_SCANCODE_RIGHT] || state[SDL_SCANCODE_D])
    {
        double oldDirX = dirX;
        dirX = dirX * cos(-rotSpeed * deltaTime) - dirY * sin(-rotSpeed * deltaTime);
        dirY = oldDirX * sin(-rotSpeed * deltaTime) + dirY * cos(-rotSpeed * deltaTime);
        double oldPlaneX = planeX;
        planeX = planeX * cos(-rotSpeed * deltaTime) - planeY * sin(-rotSpeed * deltaTime);
        planeY = oldPlaneX * sin(-rotSpeed * deltaTime) + planeY * cos(-rotSpeed * deltaTime);
    }
    if (state[SDL_SCANCODE_LEFT] || state[SDL_SCANCODE_A])
    {
        double oldDirX = dirX;
        dirX = dirX * cos(rotSpeed * deltaTime) - dirY * sin(rotSpeed * deltaTime);
        dirY = oldDirX * sin(rotSpeed * deltaTime) + dirY * cos(rotSpeed * deltaTime);
        double oldPlaneX = planeX;
        planeX = planeX * cos(rotSpeed * deltaTime) - planeY * sin(rotSpeed * deltaTime);
        planeY = oldPlaneX * sin(rotSpeed * deltaTime) + planeY * cos(rotSpeed * deltaTime);
    }
    if (state[SDL_SCANCODE_SPACE] && !isJumping)
    {
        verticalSpeed = JUMP_FORCE;
        isJumping = true;
    }

    // Apply gravity and vertical speed
    playerZ += verticalSpeed * deltaTime;
    verticalSpeed += GRAVITY * deltaTime;

    // Ensure the player lands back on the ground
    if (playerZ <= 0.0)
    {
        playerZ = 0.0;
        verticalSpeed = 0.0;
        isJumping = false;
    }
}


void mainloop()
{
    deltaTime = (clock() - lastUpdate) / 1000.0f;
    lastUpdate = clock();
    Update();
    Render();
}

int main()
{
    // Initialise SDL
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Raycasting Demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, 0);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    emscripten_set_main_loop(mainloop, 0, 1);


    return 0;
}
