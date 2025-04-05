from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import os
import requests

class SeleniumScraper:
    def __init__(self, url, save_dir="./water_images"):
        self.url = url
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

    def scroll_until_end(self, driver, scroll_pause_time=2, max_attempts=10):
        """Scroll down until the page stops growing or we hit a max number of attempts."""
        last_height = driver.execute_script("return document.body.scrollHeight")
        attempts = 0
        while attempts < max_attempts:
            driver.execute_script("window.scrollTo(0, document.body.scrollHeight);")
            time.sleep(scroll_pause_time)
            new_height = driver.execute_script("return document.body.scrollHeight")
            if new_height == last_height:
                break
            last_height = new_height
            attempts += 1

    def extract_image_url(self, img):
        """
        Try several attributes to get a valid image URL.
        Checks 'src', then 'data-src', then 'srcset' (taking the first URL).
        """
        img_url = img.get_attribute("src")
        if img_url and img_url.startswith("http"):
            return img_url

        img_url = img.get_attribute("data-src")
        if img_url and img_url.startswith("http"):
            return img_url

        srcset = img.get_attribute("srcset")
        if srcset:
            # srcset is a comma-separated list. Take the first URL.
            parts = srcset.split(",")
            if parts:
                first_part = parts[0].strip().split(" ")[0]
                if first_part.startswith("http"):
                    return first_part
        return None

    def fetch_images(self):
        options = webdriver.ChromeOptions()
        # For debugging, you might remove headless mode:
        # options.add_argument("--headless")
        options.add_argument("--disable-gpu")
        options.add_argument("--no-sandbox")

        service = Service(ChromeDriverManager().install())
        driver = webdriver.Chrome(service=service, options=options)
        driver.get(self.url)

        # Wait until at least one image element is present
        try:
            WebDriverWait(driver, 10).until(
                EC.presence_of_element_located((By.TAG_NAME, "img"))
            )
        except Exception as e:
            print("No images loaded within the timeout period.")
            driver.quit()
            return

        # Scroll until the page stops growing to trigger lazy loading
        self.scroll_until_end(driver, scroll_pause_time=2, max_attempts=10)
        time.sleep(2)  # Wait for any final lazy-load images to finish loading

        images = driver.find_elements(By.TAG_NAME, "img")
        print(f"Found {len(images)} image elements on the page.")

        saved = 0
        for i, img in enumerate(images):
            # Ensure image is in view (helpful for some lazy loaders)
            driver.execute_script("arguments[0].scrollIntoView(true);", img)
            time.sleep(1)

            img_url = self.extract_image_url(img)
            if img_url:
                try:
                    response = requests.get(img_url, timeout=10)
                    if response.status_code == 200:
                        img_path = os.path.join(self.save_dir, f"image_{i}.jpg")
                        with open(img_path, "wb") as f:
                            f.write(response.content)
                        print(f"Saved image {i}: {img_path}")
                        saved += 1
                    else:
                        print(f"Image {i} URL returned status code {response.status_code}")
                except Exception as e:
                    print(f"Failed to download image {i} from {img_url}: {e}")
            else:
                print(f"No valid URL found for image {i}.")

        print(f"Total images saved: {saved}")
        driver.quit()

# Example usage
url = "https://www.pexels.com/search/water%20texture/"
scraper = SeleniumScraper(url)
scraper.fetch_images()
