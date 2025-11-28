Below is a cohesive, article-style guide for hosting multiple sites on one Apache2 server—each bound to its own port—on Ubuntu 22.04. It walks you through every step, explains why it’s necessary, and shows you exactly what configuration to use so that browsing and refreshing never return a 404.

---

## Running Multiple Sites on One Apache Server

### Introduction

When you want to host more than one application on a single server, binding each to its own TCP port is an easy, low-overhead solution. Instead of acquiring additional IP addresses or using complex reverse-proxy setups, Apache can simply “listen” on extra ports and serve different sites from separate virtual-host configurations. This article shows you how to set it up end-to-end on Ubuntu 22.04, so that each site has its own directory, log files, and port—without ever seeing that frustrating 404 on page reload.

---

## 1. Preparing Apache to Listen on New Ports

By default, Apache listens on ports 80 (HTTP) and 443 (HTTPS). To add two more—say 8080 for *site1* and 9090 for *site2*—you must edit the global ports file:

```bash
sudo nano /etc/apache2/ports.conf
```

At the bottom, add:

```apache
Listen 8080
Listen 9090
```

Save and close. This tells Apache’s core to accept connections on those ports.

---

## 2. Defining Virtual Hosts

Each site needs its own virtual-host file in `/etc/apache2/sites-available`. These files must match the names you’ll enable with `a2ensite`, so we’ll call them `site1.conf` and `site2.conf`.

### 2.1 site1 (port 8080)

Create `/etc/apache2/sites-available/site1.conf`:

```apache
<VirtualHost *:8080>
    ServerName site1.example.com
    ServerAlias www.site1.example.com

    DocumentRoot /var/www/site1/public_html
    <Directory /var/www/site1/public_html>
        Options Indexes FollowSymLinks
        AllowOverride All
        Require all granted
        DirectoryIndex index.html
    </Directory>

    ErrorLog  ${APACHE_LOG_DIR}/site1-error.log
    CustomLog ${APACHE_LOG_DIR}/site1-access.log combined
</VirtualHost>
```

#### Highlights

* **ServerName / ServerAlias** ensure Apache picks the correct host header.
* **DocumentRoot** and `<Directory>` point exactly at where your `index.html` lives.
* **DirectoryIndex** explicitly tells Apache to serve `index.html` on directory requests.

### 2.2 site2 (port 9090)

Create `/etc/apache2/sites-available/site2.conf`:

```apache
<VirtualHost *:9090>
    ServerName site2.example.com
    ServerAlias www.site2.example.com

    DocumentRoot /var/www/site2/public_html
    <Directory /var/www/site2/public_html>
        Options Indexes FollowSymLinks
        AllowOverride All
        Require all granted
        DirectoryIndex index.html
    </Directory>

    ErrorLog  ${APACHE_LOG_DIR}/site2-error.log
    CustomLog ${APACHE_LOG_DIR}/site2-access.log combined
</VirtualHost>
```

---

## 3. Creating Document Roots

Make sure the directories exist—and contain at least an `index.html`—so Apache has something to serve:

```bash
sudo mkdir -p /var/www/site1/public_html /var/www/site2/public_html
echo "Site1 on 8080" | sudo tee /var/www/site1/public_html/index.html
echo "Site2 on 9090" | sudo tee /var/www/site2/public_html/index.html
sudo chown -R www-data:www-data /var/www/site1 /var/www/site2
```

This step ensures file permissions are correct for Apache’s `www-data` user.

---

## 4. Enabling Sites and Reloading Apache

Activate your new virtual hosts and remove the default if you don’t need it:

```bash
sudo a2ensite site1.conf site2.conf
sudo a2dissite 000-default.conf    # optional
sudo apache2ctl configtest         # check for syntax errors
sudo systemctl reload apache2      # apply the changes
```

If `configtest` passes, your server is now aware of ports 8080 and 9090 and knows which site to serve on each.

---

## 5. Opening the Firewall

If you’re using UFW on Ubuntu, permit incoming traffic on the new ports:

```bash
sudo ufw allow 8080/tcp
sudo ufw allow 9090/tcp
```

Verify with `sudo ufw status` to ensure those rules are active.

---

## 6. Testing Your Setup

On the server itself:

```bash
curl http://localhost:8080    # should return "Site1 on 8080"
curl http://localhost:9090    # should return "Site2 on 9090"
```

From any browser:

* `http://your-server-ip:8080`
* `http://your-server-ip:9090`

You’ll see each site’s index page—and refreshing will never throw a 404.

---

## 7. Troubleshooting Common Pitfalls

1. **Ports not listening**
   – Confirm `Listen` directives in `/etc/apache2/ports.conf`.
2. **Virtual-host file name mismatch**
   – `a2ensite site1.conf` must match an existing file in `sites-available`.
3. **Permissions errors**
   – Ensure `/var/www/...` is owned by `www-data:www-data`.
4. **Firewall or cloud rules**
   – Double-check UFW, iptables, or your cloud provider’s security groups.
5. **DNS or hosts file**
   – Browser requests to `site1.example.com` must resolve to this server’s IP.

---

### Conclusion

By adding a few lines to `ports.conf`, creating clear virtual-host files, and organizing your document roots, you can host as many sites as you like on different ports—with separate logs and settings—on a single Apache server. This approach keeps each application isolated, simplifies management, and avoids 404s on reload. If you need to automate these steps, you can easily wrap them into a shell script or an Ansible playbook—but the manual process outlined here is a solid foundation for any multi-site Apache deployment.
