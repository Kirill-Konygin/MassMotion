#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <cstdlib>
#include <memory>

int RunViewer()
{
    const std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
        SDL_CreateWindow("MassMotion Grid Viewer", 960, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY),
        SDL_DestroyWindow);
    if (!window)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create window: %s", SDL_GetError());
        return EXIT_FAILURE;
    }

    const std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer(
        SDL_CreateRenderer(window.get(), nullptr), SDL_DestroyRenderer);
    if (!renderer)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create renderer: %s", SDL_GetError());
        return EXIT_FAILURE;
    }

    while (true)
    {
        if (!SDL_SetRenderDrawColor(renderer.get(), 24, 28, 34, 255) || !SDL_RenderClear(renderer.get()) ||
            !SDL_RenderPresent(renderer.get()))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not render frame: %s", SDL_GetError());
            return EXIT_FAILURE;
        }

        // The empty viewer redraws on events and sleeps while idle.
        SDL_Event event;
        if (!SDL_WaitEvent(&event))
        {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not wait for events: %s", SDL_GetError());
            return EXIT_FAILURE;
        }

        if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED ||
            (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
        {
            return EXIT_SUCCESS;
        }
    }
}

int main(int, char**)
{
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not initialize SDL: %s", SDL_GetError());
        SDL_Quit();
        return EXIT_FAILURE;
    }

    const int result = RunViewer();
    SDL_Quit();
    return result;
}
